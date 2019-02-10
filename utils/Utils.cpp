//
// Created by ritvik on 17.01.19.
//

#include "Utils.h"

// --------------------------------------------------------------------------------------------------------------------
// MISCELLANEOUS UTILITY FUNCTIONS
// --------------------------------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------------------------------
// FUNCTIONS FOR PHIDGET IMU
// --------------------------------------------------------------------------------------------------------------------

// Check if the IMU should be used
void checkIMU(const std::string &strSettingsPath, int activate){

    cv::FileStorage fSettings(strSettingsPath, cv::FileStorage::READ);
    activate = fSettings["activateIMU"];

}

// Used to display any error messages from the IMU during initialization
void DisplayPhidgetError(PhidgetReturnCode returnCode, char *message) {
    PhidgetReturnCode prc; //Used to catch error codes from each Phidget function call
    const char *error;

    fprintf(stderr, "Runtime Error -> %s\n\t", message);

    prc = Phidget_getErrorDescription(returnCode, &error);
    if (prc != EPHIDGET_OK) {
        DisplayPhidgetError(prc, "Getting ErrorDescription");
        return;
    }

    fprintf(stderr, "Desc: %s\n", error);

    if (returnCode == EPHIDGET_WRONGDEVICE) {
        fprintf(stderr,
                "\tThis error commonly occurs when the Phidget function you are calling does not match the class of the channel that called it.\n"
                "\tFor example, you would get this error if you called a PhidgetVoltageInput_* function with a PhidgetDigitalOutput channel.");
    } else if (returnCode == EPHIDGET_NOTATTACHED) {
        fprintf(stderr,
                "\tThis error occurs when you call Phidget functions before a Phidget channel has been opened and attached.\n"
                "\tTo prevent this error, ensure you are calling the function after the Phidget has been opened and the program has verified it is attached.\n");
    } else if (returnCode == EPHIDGET_NOTCONFIGURED) {
        fprintf(stderr,
                "\tThis error code commonly occurs when you call an Enable-type function before all Must-Set Parameters have been set for the channel.\n"
                "\tCheck the API page for your device to see which parameters are labled \"Must be Set\" on the right-hand side of the list.");
    }
}

// Used to exit the program in case the IMU Initialization fails
void ExitPhidgetWithErrors(PhidgetHandle *chptr) {
    Phidget_delete(chptr);
    fprintf(stderr, "\nExiting with error(s)...\n");
    printf("Press ENTER to end program.\n");
    getchar();
    exit(1);
}

// Used to check IMU errors during initialization phase
void CheckPhidgetError(PhidgetReturnCode returnCode, char *message, PhidgetHandle *chptr) {

    if (returnCode != EPHIDGET_OK) {
        DisplayPhidgetError(returnCode, message);
        ExitPhidgetWithErrors(chptr);
    }
}

// Used to check if the channel was opened properly or not in order to get the IMU readings
void CheckOpenError(PhidgetReturnCode e, PhidgetHandle *chptr) {
    PhidgetReturnCode prc; //Used to catch error codes from each Phidget function call
    Phidget_ChannelClass channelClass;
    int isRemote;

    if (e == EPHIDGET_OK)
        return;

    DisplayPhidgetError(e, "Opening Phidget Channel");
    if (e == EPHIDGET_TIMEOUT) {
        fprintf(stderr, "\nThis error commonly occurs if your device is not connected as specified, "
                        "or if another program is using the device, such as the Phidget Control Panel.\n\n"
                        "If your Phidget has a plug or terminal block for external power, ensure it is plugged in and powered.\n");

        prc = Phidget_getChannelClass(*chptr, &channelClass);
        CheckPhidgetError(prc, "Getting ChannelClass", chptr);

        if (channelClass != PHIDCHCLASS_VOLTAGEINPUT
            && channelClass != PHIDCHCLASS_VOLTAGERATIOINPUT
            && channelClass != PHIDCHCLASS_DIGITALINPUT
            && channelClass != PHIDCHCLASS_DIGITALOUTPUT) {
            fprintf(stderr, "\nIf you are trying to connect to an analog sensor, you will need to use the "
                            "corresponding VoltageInput or VoltageRatioInput API with the appropriate SensorType.\n");
        }

        prc = Phidget_getIsRemote(*chptr, &isRemote);
        CheckPhidgetError(prc, "Getting IsRemote", chptr);

        if (isRemote)
            fprintf(stderr,
                    "\nEnsure the Phidget Network Server is enabled on the machine the Phidget is plugged into.\n");
    }

    ExitPhidgetWithErrors(chptr);
}

// --------------------------------------------------------------------------------------------------------------------
// FUNCTIONS FOR CALCULATIONS
// --------------------------------------------------------------------------------------------------------------------

// Fast Inverse Square Root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// Converts a given Rotation Matrix to Euler angles
// Convention used is Y-Z-X Tait-Bryan angles
// Reference code implementation:
// https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/index.htm
cv::Mat rot2euler(const cv::Mat &rotationMatrix) {
    cv::Mat euler(3, 1, CV_64F);

    double m00 = rotationMatrix.at<double>(0, 0);
    double m02 = rotationMatrix.at<double>(0, 2);
    double m10 = rotationMatrix.at<double>(1, 0);
    double m11 = rotationMatrix.at<double>(1, 1);
    double m12 = rotationMatrix.at<double>(1, 2);
    double m20 = rotationMatrix.at<double>(2, 0);
    double m22 = rotationMatrix.at<double>(2, 2);

    double bank, attitude, heading;

    // Assuming the angles are in radians.
    if (m10 > 0.998) { // singularity at north pole
        bank = 0;
        attitude = CV_PI / 2;
        heading = atan2(m02, m22);
    } else if (m10 < -0.998) { // singularity at south pole
        bank = 0;
        attitude = -CV_PI / 2;
        heading = atan2(m02, m22);
    } else {
        bank = atan2(-m12, m11);
        attitude = asin(m10);
        heading = atan2(-m20, m00);
    }

    euler.at<double>(0) = bank;
    euler.at<double>(1) = attitude;
    euler.at<double>(2) = heading;

    return euler;
}

// Converts a given Euler angles to Rotation Matrix
// Convention used is Y-Z-X Tait-Bryan angles
// Reference:
// https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToMatrix/index.htm
cv::Mat euler2rot(const cv::Mat &euler) {
    cv::Mat rotationMatrix(3, 3, CV_64F);

    double bank = euler.at<double>(0);
    double attitude = euler.at<double>(1);
    double heading = euler.at<double>(2);

    // Assuming the angles are in radians.
    double ch = cos(heading);
    double sh = sin(heading);
    double ca = cos(attitude);
    double sa = sin(attitude);
    double cb = cos(bank);
    double sb = sin(bank);

    double m00, m01, m02, m10, m11, m12, m20, m21, m22;

    m00 = ch * ca;
    m01 = sh * sb - ch * sa * cb;
    m02 = ch * sa * sb + sh * cb;
    m10 = sa;
    m11 = ca * cb;
    m12 = -ca * sb;
    m20 = -sh * ca;
    m21 = sh * sa * cb + ch * sb;
    m22 = -sh * sa * sb + ch * cb;

    rotationMatrix.at<double>(0, 0) = m00;
    rotationMatrix.at<double>(0, 1) = m01;
    rotationMatrix.at<double>(0, 2) = m02;
    rotationMatrix.at<double>(1, 0) = m10;
    rotationMatrix.at<double>(1, 1) = m11;
    rotationMatrix.at<double>(1, 2) = m12;
    rotationMatrix.at<double>(2, 0) = m20;
    rotationMatrix.at<double>(2, 1) = m21;
    rotationMatrix.at<double>(2, 2) = m22;

    return rotationMatrix;
}

std::string FloatToString ( float Number ) {
       std::ostringstream ss;
       ss << int(Number);
       return ss.str();
}

// --------------------------------------------------------------------------------------------------------------------
// FUNCTIONS FOR THE KALMAN FILTER
// --------------------------------------------------------------------------------------------------------------------

// Initialize the Kalman Filter
void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt, int flag) {
    KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter
    if(flag == 1){
        // This is used for the real time device with the camera and the IMU
        cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
        cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-4));   // set measurement noise
    } else{
        // This is used on the PennState data for the compass
        cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-7));       // set process noise
        cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(0.05));   // set measurement noise
    }
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance
    /* DYNAMIC MODEL */
    //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]
    // position
    KF.transitionMatrix.at<double>(0, 3) = dt;
    KF.transitionMatrix.at<double>(1, 4) = dt;
    KF.transitionMatrix.at<double>(2, 5) = dt;
    KF.transitionMatrix.at<double>(3, 6) = dt;
    KF.transitionMatrix.at<double>(4, 7) = dt;
    KF.transitionMatrix.at<double>(5, 8) = dt;
    KF.transitionMatrix.at<double>(0, 6) = 0.5 * pow(dt, 2);
    KF.transitionMatrix.at<double>(1, 7) = 0.5 * pow(dt, 2);
    KF.transitionMatrix.at<double>(2, 8) = 0.5 * pow(dt, 2);
    // orientation
    KF.transitionMatrix.at<double>(9, 12) = dt;
    KF.transitionMatrix.at<double>(10, 13) = dt;
    KF.transitionMatrix.at<double>(11, 14) = dt;
    KF.transitionMatrix.at<double>(12, 15) = dt;
    KF.transitionMatrix.at<double>(13, 16) = dt;
    KF.transitionMatrix.at<double>(14, 17) = dt;
    KF.transitionMatrix.at<double>(9, 15) = 0.5 * pow(dt, 2);
    KF.transitionMatrix.at<double>(10, 16) = 0.5 * pow(dt, 2);
    KF.transitionMatrix.at<double>(11, 17) = 0.5 * pow(dt, 2);
    /* MEASUREMENT MODEL */
    //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
    KF.measurementMatrix.at<double>(0, 0) = 1;  // x
    KF.measurementMatrix.at<double>(1, 1) = 1;  // y
    KF.measurementMatrix.at<double>(2, 2) = 1;  // z
    KF.measurementMatrix.at<double>(3, 9) = 1;  // roll
    KF.measurementMatrix.at<double>(4, 10) = 1; // pitch
    KF.measurementMatrix.at<double>(5, 11) = 1; // yaw
}

// Fill the measurements vector
void fillMeasurements(cv::Mat &measurements,
                      const cv::Mat &translation_measured, const cv::Mat &rotation_measured) {
    // Convert rotation matrix to euler angles
    cv::Mat measured_eulers(3, 1, CV_64F);
    measured_eulers = rot2euler(rotation_measured);
    // Set measurement to predict
    measurements.at<double>(0) = translation_measured.at<double>(0); // x
    measurements.at<double>(1) = translation_measured.at<double>(1); // y
    measurements.at<double>(2) = translation_measured.at<double>(2); // z
    measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
    measurements.at<double>(4) = -1.0 * measured_eulers.at<double>(1);      // pitch
    measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw
}

void fillMeasurementsPennState(cv::Mat &measurements,
        const cv::Mat &translation_measured, const cv::Mat &rotation_measured) {
    fillMeasurements(measurements, translation_measured, rotation_measured);
    measurements.at<double>(4) = -1.0 * measurements.at<double>(4);  // reverts again pitch
}

// Update the Kalman Filter
void updateKalmanFilter(cv::KalmanFilter &KF, cv::Mat &measurement,
                        cv::Mat &translation_estimated, cv::Mat &rotation_estimated) {
    // First predict, to update the internal statePre variable
    cv::Mat prediction = KF.predict();
    // std::cout << "Prediction: " << prediction.at<double>(0) << " " << prediction.at<double>(1) << " " << prediction.at<double>(2) << std::endl;

    // The "correct" phase that is going to use the predicted value and our measurement
    cv::Mat estimated = KF.correct(measurement);
    // std::cout << "Estimation: " << estimated.at<double>(0) << " " << estimated.at<double>(1) << " " << estimated.at<double>(2) << std::endl;

    // Estimated translation
    translation_estimated.at<double>(0) = estimated.at<double>(0);
    translation_estimated.at<double>(1) = estimated.at<double>(1);
    translation_estimated.at<double>(2) = estimated.at<double>(2);

    // Estimated euler angles
    cv::Mat eulers_estimated(3, 1, CV_64F);
    eulers_estimated.at<double>(0) = estimated.at<double>(9);
    eulers_estimated.at<double>(1) = estimated.at<double>(10);
    eulers_estimated.at<double>(2) = estimated.at<double>(11);
    // Convert estimated quaternion to rotation matrix
    rotation_estimated = euler2rot(eulers_estimated);
}

// --------------------------------------------------------------------------------------------------------------------
// FUNCTIONS FOR DRAWING THE CAMERA
// --------------------------------------------------------------------------------------------------------------------

// Create the OpenGL Camera Matrix
void getCurrentOpenGLCameraMatrix(cv::Mat Rwc, cv::Mat twc, pangolin::OpenGlMatrix &M) {
    M.m[0] = Rwc.at<double>(0, 0);
    M.m[1] = Rwc.at<double>(1, 0);
    M.m[2] = Rwc.at<double>(2, 0);
    M.m[3] = 0.0;

    M.m[4] = Rwc.at<double>(0, 1);
    M.m[5] = Rwc.at<double>(1, 1);
    M.m[6] = Rwc.at<double>(2, 1);
    M.m[7] = 0.0;

    M.m[8] = Rwc.at<double>(0, 2);
    M.m[9] = Rwc.at<double>(1, 2);
    M.m[10] = Rwc.at<double>(2, 2);
    M.m[11] = 0.0;

    M.m[12] = twc.at<double>(0);
    M.m[13] = twc.at<double>(1);
    M.m[14] = twc.at<double>(2);
    M.m[15] = 1.0;
}

// Draw the current camera
void drawCurrentCamera(pangolin::OpenGlMatrix &Twc) {
    const float &w = 0.1;
    const float h = w * 1.5;
    const float z = w;

    glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

    glLineWidth(1);
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINES);

    glVertex3f(Twc.m[12], Twc.m[13], Twc.m[14]);
    glVertex3f((Twc.m[12]+w), (Twc.m[13]+h), (Twc.m[14]+z));
    glVertex3f(Twc.m[12], Twc.m[13], Twc.m[14]);
    glVertex3f((Twc.m[12]+w), (Twc.m[13]-h), (Twc.m[14]+z));
    glVertex3f(Twc.m[12], Twc.m[13], Twc.m[14]);
    glVertex3f((Twc.m[12]-w), (Twc.m[13]-h), (Twc.m[14]+z));
    glVertex3f(Twc.m[12], Twc.m[13], Twc.m[14]);
    glVertex3f((Twc.m[12]-w), (Twc.m[13]+h), (Twc.m[14]+z));

    glVertex3f((Twc.m[12]+w), (Twc.m[13]+h), (Twc.m[14]+z));
    glVertex3f((Twc.m[12]+w), (Twc.m[13]-h), (Twc.m[14]+z));

    glVertex3f((Twc.m[12]-w), (Twc.m[13]+h), (Twc.m[14]+z));
    glVertex3f((Twc.m[12]-w), (Twc.m[13]-h), (Twc.m[14]+z));

    glVertex3f((Twc.m[12]-w), (Twc.m[13]+h), (Twc.m[14]+z));
    glVertex3f((Twc.m[12]+w), (Twc.m[13]+h), (Twc.m[14]+z));

    glVertex3f((Twc.m[12]-w), (Twc.m[13]-h), (Twc.m[14]+z));
    glVertex3f((Twc.m[12]+w), (Twc.m[13]-h), (Twc.m[14]+z));

    glEnd();

    glPopMatrix();
}

// Draw the Axis
void drawAxis() {

    glLineWidth(1);
    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_LINES);

    glVertex3f(0, 0, 0);
    glVertex3f(5, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 5, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 5);

    glEnd();
}

// --------------------------------------------------------------------------------------------------------------------
// FUNCTIONS FOR ARUCO TRACKING
// --------------------------------------------------------------------------------------------------------------------

void cameraParameters(const std::string &strSettingsPath, cv::Mat &cameraMatrix, cv::Mat &distortionCoeffs){

    cv::FileStorage fSettings(strSettingsPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(cameraMatrix);

    cv::Mat DistCoef(5,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    DistCoef.at<float>(4) = fSettings["Camera.k3"];
    DistCoef.copyTo(distortionCoeffs);

}


void detectArucoMarkers(cv::Mat &frame, float markerLength, const cv::Mat &cameraMatrix, const cv::Mat &distortionCoeffs,
        std::vector<int> &ids, std::vector<std::vector<cv::Point2f> > &corners,
        std::vector<cv::Vec3d> &rotationVectors, std::vector<cv::Vec3d> &translationVectors){

    // Aruco Dictionary
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    cv::aruco::detectMarkers(frame, dictionary, corners, ids);

    if(ids.size() > 0){
        cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix,
                distortionCoeffs, rotationVectors, translationVectors);
    }

}

// Use the 3d position of the Camera with respec to the Aruco Marker
// and calculate its position in the world coordinates
void calculatePositions(cv::Vec3d &trCamAruco, cv::Mat &rotCamWorldSpace, 
        cv::Mat &trCamWorldSpace, cv::Mat &cameraMatrix, cv::Mat &trArucoWorld){

    // Converting the input matrices and vectors from CV to Eigen
    Eigen::Vector3d arucoTrVecEigen;
    arucoTrVecEigen.x() = trCamAruco[0];
    arucoTrVecEigen.y() = trCamAruco[1];
    arucoTrVecEigen.z() = trCamAruco[2];
    // std::cout << "arucoTrVecEigen: " << arucoTrVecEigen << std::endl;

    Eigen::Matrix<double, 3, 3> rotCamWorldSpaceEigen;
    cv2eigen(rotCamWorldSpace, rotCamWorldSpaceEigen);
    // std::cout << "rotCamWorldSpaceEigen: " << rotCamWorldSpaceEigen << std::endl;

    Eigen::Vector3d trCamWorldSpaceEigen;
    trCamWorldSpaceEigen.x() = trCamWorldSpace.at<float>(0,0);
    trCamWorldSpaceEigen.y() = trCamWorldSpace.at<float>(1,0);
    trCamWorldSpaceEigen.z() = trCamWorldSpace.at<float>(2,0);
    // std::cout << "trCamWorldSpaceEigen: " << trCamWorldSpaceEigen << std::endl;

    Eigen::Matrix<double, 3, 3> cameraMatrixEigen;
    cv2eigen(cameraMatrix, cameraMatrixEigen);

    // Initializing a vector to store the intermediate calculation steps
    Eigen::Vector3d calculationsVector;
    
    // Invert the camera matrix
    Eigen::Matrix3d cameraInv = cameraMatrixEigen.inverse();

    calculationsVector = (cameraInv * arucoTrVecEigen);
    calculationsVector = (rotCamWorldSpaceEigen * calculationsVector) + trCamWorldSpaceEigen;
    // calculationsVector = (rotCamWorldSpaceEigen * arucoTrVecEigen) + trCamWorldSpaceEigen;

    trArucoWorld.at<float>(0,0) = calculationsVector.x();
    trArucoWorld.at<float>(1,0) = calculationsVector.y();
    trArucoWorld.at<float>(2,0) = calculationsVector.z();

    // std::cout << calculationsVector << std::endl;
}

// Calculate Camera World Coordinates
void calculateWorldCamera(cv::Mat &cameraMatrix, cv::Mat &rotCamWorldSpace, cv::Mat &trCamWorldSpace, cv::Mat &trWorldCam){

    Eigen::Matrix<double, 3, 3> cameraMatrixEigen;
    cv2eigen(cameraMatrix, cameraMatrixEigen);
    
    // Invert the camera matrix
    Eigen::Matrix3d cameraInv = cameraMatrixEigen.inverse();

    Eigen::Matrix<double, 3, 3> rotCamWorldSpaceEigen;
    cv2eigen(rotCamWorldSpace, rotCamWorldSpaceEigen);
    // std::cout << "rotCamWorldSpaceEigen: " << rotCamWorldSpaceEigen << std::endl;

    Eigen::Vector3d trCamWorldSpaceEigen;
    trCamWorldSpaceEigen.x() = trCamWorldSpace.at<float>(0,0);
    trCamWorldSpaceEigen.y() = trCamWorldSpace.at<float>(1,0);
    trCamWorldSpaceEigen.z() = trCamWorldSpace.at<float>(2,0);
    // std::cout << "trCamWorldSpaceEigen: " << trCamWorldSpaceEigen << std::endl;
    
    // Initializing a vector to store the intermediate calculation steps
    Eigen::Vector3d calculationsVector;
    // calculationsVector = rotCamWorldSpaceEigen * trCamWorldSpaceEigen;
    calculationsVector = cameraInv * trCamWorldSpaceEigen;
    // std::cout << calculationsVector.x() << "   " << calculationsVector.y() << "   " << calculationsVector.z() << std::endl;

    trWorldCam.at<float>(0,0) = calculationsVector.x();
    trWorldCam.at<float>(1,0) = calculationsVector.y();
    trWorldCam.at<float>(2,0) = calculationsVector.z();

}

// --------------------------------------------------------------------------------------------------------------------
// FUNCTIONS FOR DRAWING ARROW
// --------------------------------------------------------------------------------------------------------------------

// Calculate the angle for the object
float calculateAngle(cv::Mat &cameraMatrix, cv::Mat &rotCamWorldSpace, cv::Mat &trCamWorldSpace, 
        cv::Mat &markerPosition){

    Eigen::Matrix<double, 3, 3> cameraMatrixEigen;
    cv2eigen(cameraMatrix, cameraMatrixEigen);

    // Invert the camera matrix
    Eigen::Matrix3d cameraInv = cameraMatrixEigen.inverse();

    Eigen::Matrix<double, 3, 3> rotCamWorldSpaceEigen;
    cv2eigen(rotCamWorldSpace, rotCamWorldSpaceEigen);

    Eigen::Vector3d trCamWorldSpaceEigen;
    trCamWorldSpaceEigen.x() = trCamWorldSpace.at<float>(0,0);
    trCamWorldSpaceEigen.y() = trCamWorldSpace.at<float>(1,0);
    trCamWorldSpaceEigen.z() = trCamWorldSpace.at<float>(2,0);

    //Normal Vector to the camera
    Eigen::Vector3d camNormal;
    camNormal.x() = 0;
    camNormal.y() = 0;
    camNormal.z() = 1;

    //Translate the normal vector to the world space
    camNormal = (cameraInv * camNormal);
    camNormal = (rotCamWorldSpaceEigen * camNormal) + trCamWorldSpaceEigen;

    // Calculate the vector between the camera and the object
    Eigen::Vector3d markerPositionEigen;
    markerPositionEigen.x() = markerPosition.at<float>(0,0);
    markerPositionEigen.y() = markerPosition.at<float>(1,0);
    markerPositionEigen.z() = markerPosition.at<float>(2,0);

    Eigen::Vector3d camObject;
    camObject = camNormal - markerPositionEigen;

    // Calculate the angle
    float angle = acos(camObject.dot(camNormal)/(camObject.norm() * camNormal.norm()));
    Eigen::Vector3d crossP = camNormal.cross(camObject);
    // std::cout << "crossP: " << crossP << std::endl; 
    // Eigen::Vector3d n = crossP / crossP.norm();
    // float angle = atan2(n.dot(crossP), camNormal.dot(camObject));
    // float angle = atan2((camNormal.cross(camObject)).norm(), camNormal.dot(camObject));
    // float dot = (camNormal.x() * camObject.x()) + (camNormal.y() * camObject.y()) + (camNormal.z() * camObject.z());
    // float det = (camNormal.x() * camObject.y() * crossP.z()) + (camObject.x() * crossP.y() * camNormal.z());
    // det = det + (crossP.x() * camNormal.y() * camObject.z()) - (camNormal.z() * camObject.y() * crossP.x());
    // det = det - (camObject.z() * crossP.y() * camNormal.x()) - (crossP.z() * camNormal.y() * camObject.x());
    // float angle = atan2(det, dot);
    angle = angle * 180 / M_PI;
    angle = angle * 4.5;
    if(crossP.y() > 0){
        angle = -1.0 * angle;
    }
    // std::cout << "calculateAngle: " << angle << std::endl;

    return angle;

}

// Insert arrow into Image
void insertArrow(cv::Mat &sourceImage, cv::Mat &arrow, cv::Mat &arrowBgOr, int angle, cv::Mat &destImage){
    if(angle < 0){
            angle = 360 + angle;
    }
    cv::Point2f pic_c(arrowBgOr.cols/2., arrowBgOr.rows/2.);
    cv::Mat r = cv::getRotationMatrix2D(pic_c, int(angle), 1.0);
    cv::warpAffine(arrowBgOr, arrow, r, arrowBgOr.size());
    cv::Rect roi(0,0,80,80);
    cv::resize(arrow, arrow, cv::Size(80, 80), 0, 0);
    cv::Mat insetImage;
    insetImage = sourceImage(roi);
    arrow.copyTo(insetImage);
    destImage = sourceImage;
}
