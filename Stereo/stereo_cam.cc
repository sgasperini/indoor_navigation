/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*
* Run this program: ./stereo_cam /home/ritvik/indoor_navigation/Vocabulary/ORBvoc.txt /home/ritvik/indoor_navigation/Stereo/RealSense.yaml
*/

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <time.h>
#include <librealsense2/rs.hpp>
#include <cstdlib>
#include <System.h>
#include "Utils.h"

using namespace std;
using namespace cv;
using namespace Eigen;

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq 100.0f      // sample frequency in Hz
#define twoKpDef (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef (2.0f * 0.0f) // 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;                                           // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;                                           // 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                 // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

//===================================================================================================
// Function Definitions

//---------------------------------------------------------------------------------------------------
// IMU algorithm update: The update algorithm that gets called internally
// within MahonyAHRS update if only accelerometer and gyroscope readings
// are available

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// MahonyAHRS Algorithm Update : This algorithm takes the readings of the
// gyroscope, accelerometer and magnetometer (if available)
// and corrects gyroscope readings and returns a quaternion

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//====================================================================================================

int main(int argc, char **argv)
{

    //Initializing vectors to hold the translations and rotations
    std::vector<Eigen::Quaterniond> quaternions;
    std::vector<cv::Mat> rotations;
    std::vector<cv::Mat> translations;

    //Check if the IMU should be used
    int activate;
    checkIMU(argv[2], activate);

    //Initializing Scale Factor Matrix and Bias for Accelerometer and Gyroscope
    Matrix3d scaleFactor;
    scaleFactor << 1.0038, 0.0004, 0.0536,
        0.0004, 0.9892, -0.0061,
        0.0536, -0.0061, 0.9967;
    Vector3d biasAcc(-0.0200, -0.0033, 0.0086);
    //    Vector3d biasGyr(-0.17331, 0.42816, -0.27329);
    Vector3d biasGyr(-0.3768, 0.5205, -0.0332);
    //    Vector3d biasGyr(-0.3944, 0.3908, 0.07);
    

    float sensX = 10.0;
    float sensY = 10.0;
    float sensZ = 10.0;

    int dataIntervalAcc = 1;
    int dataIntervalGyr = 1;

    //Initializations for Accelerometer and Gyroscope
    PhidgetAccelerometerHandle chAcc = NULL;
    PhidgetReturnCode prcAcc;
    prcAcc = PhidgetAccelerometer_create(&chAcc);
    if(activate == 1){
        CheckPhidgetError(prcAcc, "Creating Channel", (PhidgetHandle *)&chAcc);
        prcAcc = Phidget_openWaitForAttachment((PhidgetHandle)chAcc, 5000);
        CheckOpenError(prcAcc, (PhidgetHandle *)&chAcc);
        dataIntervalAcc = PhidgetAccelerometer_setDataInterval((PhidgetAccelerometerHandle)chAcc, 100);
    }

    PhidgetGyroscopeHandle chGyr = NULL;
    PhidgetReturnCode prcGyr;
    prcGyr = PhidgetGyroscope_create(&chGyr);
    if(activate == 1){
        CheckPhidgetError(prcGyr, "Creating Channel", (PhidgetHandle *)&chGyr);
        prcGyr = Phidget_openWaitForAttachment((PhidgetHandle)chGyr, 5000);
        CheckOpenError(prcGyr, (PhidgetHandle *)&chGyr);
        dataIntervalGyr = PhidgetGyroscope_setDataInterval((PhidgetGyroscopeHandle)chGyr, 100);   
    }
    
    // RealSense Pipeline encapsulating actual device and sensors
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_INFRARED, 1);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2);

    // Start streaming from the pipeline
    rs2::pipeline_profile selection = pipe.start(cfg);
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    // Disable the projection pattern
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
    }

    // Check the starting time
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, false);
    // SLAM.StartViewer();

    //Initializations for viewing
    pangolin::CreateWindowAndBind("Main", 640, 480);
    glEnable(GL_DEPTH_TEST);
    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
        //            pangolin::ModelViewLookAt(1,-1,3, 0,0,0, pangolin::AxisY)
        //            pangolin::ModelViewLookAt(0, 0, -3, 0, 0, 0, pangolin::AxisY)
        pangolin::ModelViewLookAt(0, 3, 0, 0, 0, 0, pangolin::AxisZ));
    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
                                .SetHandler(&handler);

    //Initializing the Kalman Filter
    cv::KalmanFilter KF;
    int nStates = 18;
    int nMeasurements = 6;
    int nInputs = 0;
    double dt = 0.1;
    int flag = 1;
    initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt, flag);
    Mat measurements(nMeasurements, 1, CV_64F);
    measurements.setTo(Scalar(0));
    //std::cout << "Reached here 0" << std::endl;

    int frameCount = 0;
    int imuCount = 0;

    //    // Empty Quaternion to hold the final rotation
    //    cv::Mat cumulRot = cv::Mat::eye(3, 3, CV_64F);
    //    Eigen::Matrix<double, 3, 3> rotMatTot;
    //    cv2eigen(cumulRot, rotMatTot);
    //    Eigen::Quaterniond qTotal(rotMatTot);

    //Rotation for device space
    Matrix3d spaceRot;
    spaceRot << 1.0, 0.0, 0.0,
        0.0, 0.866, -0.5,
        0.0, 0.5, 0.866;
    Eigen::Quaterniond spaceRotQuat(spaceRot);
    spaceRotQuat.normalize();

    // Intializing the data structure to store the detected objects
    std::vector<ObjectPosition> objects;
    std::set<int> objectIds;
    // Initializing other Aruco Related things
    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs(5, 1, CV_32F);
    cameraParameters(argv[2], cameraMatrix, distortionCoeffs);
    //    std::cout << cameraMatrix << std::endl;
    //    std::cout << distortionCoeffs << std::endl;
    float markerLength = 0.045;
    // Aruco Dictionary
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // Arrow Visualization
    cv::Mat arrow = cv::imread("/home/ritvik/indoor_navigation/arrow.png", CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat arrowBgOr = arrow.clone();

    //for(;;){
    while (!pangolin::ShouldQuit())
    {

        frameCount++;
        rs2::frameset data = pipe.wait_for_frames();
        rs2::frame ir1 = data.get_infrared_frame(1);
        rs2::frame ir2 = data.get_infrared_frame(2);

        //Find the size of the frame
        int widthIR1 = ir1.as<rs2::video_frame>().get_width();
        int heightIR1 = ir1.as<rs2::video_frame>().get_height();
        int widthIR2 = ir2.as<rs2::video_frame>().get_width();
        int heightIR2 = ir2.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the data
        Mat irFrame1(Size(widthIR1, heightIR1), CV_8UC1, (void *)ir1.get_data(), Mat::AUTO_STEP);
        Mat irFrame2(Size(widthIR2, heightIR2), CV_8UC1, (void *)ir2.get_data(), Mat::AUTO_STEP);
        Mat showFrame(irFrame1);
        cv::resize(showFrame, showFrame, cv::Size(), 0.75, 0.75);

        // Check the current time
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        // Evaluate the timestamp
        double tFrame = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        if (irFrame1.empty() || irFrame2.empty())
        {
            cerr << "Failed to read image from the camera" << endl;
            return 1;
        }

        // cv::Mat irLeftRect, irRightRect;
        //       cv::remap(irFrame1,irLeftRect,M1l,M2l,cv::INTER_LINEAR);
        //       cv::remap(irFrame2,irRightRect,M1r,M2r,cv::INTER_LINEAR);

        // Pass the images to the SLAM system
        Mat currentPose = SLAM.TrackStereo(irFrame1, irFrame2, tFrame);

        //        std::cout << "Reached here 2" << std::endl;

        // Instantiate estimated translation and rotation
        cv::Mat translation_estimated(3, 1, CV_64F);
        cv::Mat rotation_estimated(3, 3, CV_64F);

        if (!currentPose.empty())
        {

            //        	std::cout << currentPose << std::endl;
            cv::Mat rotMat = currentPose(cv::Range(0, 3), cv::Range(0, 3));
            cv::Mat transl = currentPose.rowRange(0, 3).col(3);
            // std::cout << "SLAM Translation" << transl << std::endl;
            Eigen::Matrix<double, 3, 3> rotMatE;
            cv2eigen(rotMat, rotMatE);
            Eigen::Quaterniond qCam(rotMatE);
            qCam.normalize();

            // std::cout << "Camera:" << transl.at<float>(0) << " " << transl.at<float>(1) << " " << transl.at<float>(2) << std::endl;

            // Get the rotation measured
            Eigen::Matrix3d rotationCam = qCam.toRotationMatrix();
            cv::Mat rotationCamKalman(3, 3, CV_64F);
            eigen2cv(rotationCam, rotationCamKalman);

            // Get the translation measured
            cv::Mat translationCamKalman(3, 1, CV_64F);
            translationCamKalman = transl;
            // std::cout << "Translation Input Kalman" << translationCamKalman << std::endl;

            //std::cout << "Reached here 1" << std::endl;

            // fill the measurements vector
            fillMeasurements(measurements, translationCamKalman, rotationCamKalman);

            // update the Kalman filter with good measurements
            updateKalmanFilter(KF, measurements, translation_estimated, rotation_estimated);
            translation_estimated = translationCamKalman;
            // std::cout << "Translation Output Kalman" << translation_estimated << std::endl;

            //            std::cout << "Reached here 3" << std::endl;

            // Check if there is any Object with Aruco Marker in the frame
            std::vector<int> arucoIds;
            std::vector<std::vector<cv::Point2f>> arucoCorners;
            std::vector<cv::Vec3d> arucoRotationVectors;
            std::vector<cv::Vec3d> arucoTranslationVectors;

            // If any ids are found, check if they are already present in objects
            // If the id is not already present,
            // then calculate its world coordinate and add in objects
            // Else ignore
            cv::aruco::detectMarkers(irFrame1, dictionary, arucoCorners, arucoIds);
            if (arucoIds.size() > 0)
            {
                cv::aruco::estimatePoseSingleMarkers(arucoCorners, markerLength, cameraMatrix,
                                                     distortionCoeffs, arucoRotationVectors, arucoTranslationVectors);
                for (int p = 0; p < arucoIds.size(); p++)
                {
                    std::set<int>::iterator it = objectIds.find(arucoIds[p]);
                    if (it == objectIds.end())
                    {
                        objectIds.insert(arucoIds[p]);
                        ObjectPosition pos;

                        // Calculate the world 3D coordinates of the Aruco Marker
                        cv::Mat trArucoWorld(3, 1, CV_64F);
                        calculatePositions(arucoTranslationVectors[p], rotation_estimated,
                                translation_estimated, cameraMatrix, trArucoWorld);

                        pos.id = arucoIds[p];
                        pos.translation = trArucoWorld;
                        objects.push_back(pos);
                    }
                }
            }

            rotations.push_back(rotation_estimated);
            translations.push_back(translation_estimated);
        }

        if(activate == 1){
            // Get the accelerometer and gyroscope information
            double acceleration[3];
            double angularRate[3];
            int retValAcc, retValGyr;
            Vector3d gyrsPrevious(0.0, 0.0, 0.0);

            for (int i = 0; i < 10; i++)
            {

                imuCount++;
                retValAcc = PhidgetAccelerometer_getAcceleration(chAcc, &acceleration);
                retValGyr = PhidgetGyroscope_getAngularRate(chGyr, &angularRate);
                Vector3d accl(acceleration[0], acceleration[1], acceleration[2]);
                Vector3d gyrs(angularRate[0], angularRate[1], angularRate[2]);
                accl = scaleFactor * accl;
                accl = accl - biasAcc;
                gyrs = gyrs - biasGyr;

                // Z Axis is possibly reversed, so reversing the values
                accl.z() = -1.0 * accl.z();
                gyrs.z() = -1.0 * gyrs.z();

                //                std::cout << (gyrs-gyrsPrevious) << std::endl;
                //                std::cout << std::endl;

                if ((abs(gyrs.x() - gyrsPrevious.x()) > 0.5) || (abs(gyrs.y() - gyrsPrevious.y()) > 0.5) || (abs(gyrs.z() - gyrsPrevious.z()) > 0.5))
                {
                    //	                std::cout << "Used IMU Readings" << std::endl;
                    gyrsPrevious = gyrs;

                    // Adjust the readings with the sensitivity
                    gyrs.x() = gyrs.x() / sensX;
                    gyrs.y() = gyrs.y() / sensY;
                    gyrs.z() = gyrs.z() / sensZ;

                    MahonyAHRSupdate(gyrs.x(), gyrs.y(), gyrs.z(), accl.x(), accl.y(), accl.z(), 0.0f, 0.0f, 0.0f);
                    //std::cout << "Acceleration: " << accl.x() << "  " << accl.y() << "  " << accl.z() << std::endl;
                    //                    std::cout << "Gyroscope: " << q0 << "  " << q1 << "  " << q2 << "  " << q3 << std::endl;
                    Eigen::Quaterniond qGyr;
                    qGyr.x() = q1;
                    qGyr.y() = q2;
                    qGyr.z() = q3;
                    qGyr.w() = q0;
                    //                qGyr = qGyr * spaceRotQuat;
                    //std::cout << "Gyroscope after: " << qGyr.w() << "  " << qGyr.x() << "  " << qGyr.y() << "  " << qGyr.z() << std::endl;
                    //                qTotal =  qTotal * qGyr;
                    //                    qTotal = qGyr;
                    //                    std::cout << transVec.x() << "  " << transVec.y() << "  " << transVec.z() << "  " << qTotal.x() << "  " << qTotal.y() << "  " << qTotal.z() << "  " << qTotal.w() << std::endl;

                    //Get the translation measured
                    cv::Mat translationIMUKalman(3, 1, CV_64F);
                    translationIMUKalman = translation_estimated;
                    //	            std::cout << translation_measured << std::endl;

                    // Get the rotation measured
                    Eigen::Matrix3d rotationIMU = qGyr.toRotationMatrix();
                    cv::Mat rotationIMUKalman(3, 3, CV_64F);
                    eigen2cv(rotationIMU, rotationIMUKalman);

                    //std::cout << "Reached here 1" << std::endl;

                    // fill the measurements vector
                    fillMeasurements(measurements, translationIMUKalman, rotationIMUKalman);
                    //std::cout << measurements << std::endl;

                    //std::cout << "Reached here 2" << std::endl;

                    // update the Kalman filter with good measurements
                    updateKalmanFilter(KF, measurements, translation_estimated, rotation_estimated);
                }
            }

            // Setting the angle for the arrow
            // SLAM.SetAngle(angle);

            // cv::Mat translationWorld(3, 1, CV_64F);
            // calculateWorldCamera(cameraMatrix, rotation_estimated, translation_estimated, translationWorld);

            // std::cout << "Camera:" << translation_estimated.at<float>(0) + 0.4 << "   " << translation_estimated.at<float>(1) << "   " << -1.0 * translation_estimated.at<float>(2) << std::endl;
            if (objects.size() > 0){
                // std::cout << "Object:" << objects[0].translation.at<float>(0,0) << "   " << objects[0].translation.at<float>(1,0) << "   " << objects[0].translation.at<float>(2,0) << std::endl; 
                //If object exists then calculate the angle
                float angle = calculateAngle(cameraMatrix, rotation_estimated, translation_estimated, objects[0].translation);
                // std::cout << "Angle to object: " << angle << std::endl;
                //Insert the Arrow in the frame and show the frame
                insertArrow(showFrame, arrow, arrowBgOr, angle, showFrame);
            }
            // angle = int(rand()%100);
            cv::imshow("Frame", showFrame);

            //Draw a pangolin camera
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            pangolin::OpenGlMatrix openGlProjection;
            d_cam.Activate(s_cam);
            // drawAxis();
            // for(int x =0; x < rotations.size(); x++){
            //     if(x%10 == 0){
            //         getCurrentOpenGLCameraMatrix(rotations[x], translations[x], openGlProjection);
            //         drawCurrentCamera(openGlProjection);
            //     }
            // }
            getCurrentOpenGLCameraMatrix(rotation_estimated, translation_estimated, openGlProjection);
            drawCurrentCamera(openGlProjection);
            if (objects.size() > 0)
            {
                // std::cout << objects[0].translation.at<float>(0,0) << " " << objects[0].translation.at<float>(1,0) << " " << objects[0].translation.at<float>(2,0) << std::endl;
                for (int p = 0; p < objects.size(); p++)
                {
                    glColor3f(0.0f, 1.0f, 0.0f);
                    pangolin::glDrawCross((objects[p].translation.at<float>(0,0)+0.4), objects[p].translation.at<float>(1,0),
                                    objects[p].translation.at<float>(2,0), 0.05);
                }
            }
            pangolin::FinishFrame();

            if ((char)27 == (char)waitKey(1))
                break;

        }
    }
    // Stop all the threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    cv::destroyAllWindows();

    return 0;
}
