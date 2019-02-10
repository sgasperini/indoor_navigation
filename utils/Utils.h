//
// Created by ritvik on 17.01.19.
//

#ifndef AT3DCV_UTILS_H
#define AT3DCV_UTILS_H

#include<iostream>
#include<Eigen/Dense>
#include<Eigen/Core>
#include<opencv2/core/core.hpp>
#include<opencv2/video/tracking.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/core/eigen.hpp>
#include<phidget22.h>
#include<math.h>
#include<pangolin/pangolin.h>
#include<opencv2/aruco.hpp>
#include<unistd.h>
#include<stdio.h>
#include<stdlib.h>

// --------------------------------------------------------------------------------------------------------------------
// MISCELLANEOUS UTILITY FUNCTIONS
// --------------------------------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------------------------------
// FUNCTIONS FOR PHIDGET IMU
// --------------------------------------------------------------------------------------------------------------------

// CHeck if the IMU should be used or not
void checkIMU(const std::string &strSettingsPath, int activate);

// Used to display any error messages from the IMU during initialization
void DisplayPhidgetError(PhidgetReturnCode returnCode, char *message);

// Used to exit the program in case the IMU Initialization fails
void ExitPhidgetWithErrors(PhidgetHandle *chptr);

// Used to check IMU errors during initialization phase
void CheckPhidgetError(PhidgetReturnCode returnCode, char *message, PhidgetHandle *chptr);

// Used to check if the channel was opened properly or not in order to get the IMU readings
void CheckOpenError(PhidgetReturnCode e, PhidgetHandle *chptr);

// --------------------------------------------------------------------------------------------------------------------
// FUNCTIONS FOR CALCULATIONS
// --------------------------------------------------------------------------------------------------------------------

// Fast Inverse Square Root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x);

// Converts a given Rotation Matrix to Euler angles
// Convention used is Y-Z-X Tait-Bryan angles
// Reference code implementation:
// https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/index.htm
cv::Mat rot2euler(const cv::Mat &rotationMatrix);

// Converts a given Euler angles to Rotation Matrix
// Convention used is Y-Z-X Tait-Bryan angles
// Reference:
// https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToMatrix/index.htm
cv::Mat euler2rot(const cv::Mat &euler);

// --------------------------------------------------------------------------------------------------------------------
// FUNCTIONS FOR THE KALMAN FILTER
// --------------------------------------------------------------------------------------------------------------------

// Initialize the Kalman Filter
void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt, int flag);

// Fill the measurements vector
void fillMeasurements(cv::Mat &measurements,
                      const cv::Mat &translation_measured, const cv::Mat &rotation_measured);

// Update the Kalman Filter
void updateKalmanFilter(cv::KalmanFilter &KF, cv::Mat &measurement,
                        cv::Mat &translation_estimated, cv::Mat &rotation_estimated);

// --------------------------------------------------------------------------------------------------------------------
// FUNCTIONS FOR DRAWING THE CAMERA
// --------------------------------------------------------------------------------------------------------------------

// Create the OpenGL Camera Matrix
void getCurrentOpenGLCameraMatrix(cv::Mat Rwc, cv::Mat twc, pangolin::OpenGlMatrix &M);

// Draw the current camera
void drawCurrentCamera(pangolin::OpenGlMatrix &Twc);

// Draw the Axis
void drawAxis();

// --------------------------------------------------------------------------------------------------------------------
// FUNCTIONS FOR ARUCO TRACKING
// --------------------------------------------------------------------------------------------------------------------

// Read the camera parameters from the parameter file for the Aruco Tracking
void cameraParameters(const std::string &strSettingsPath, cv::Mat &cameraMatrix, cv::Mat &distortionCoeffs);

// If any markers are present in the image, detect them and calculate their
// ids, corner positions in the image and 3d positions with respect to camera
void detectArucoMarkers(cv::Mat &frame, float markerLength, cv::Mat &cameraMatrix, cv::Mat &distortionCoeffs,
        std::vector<int> &ids, std::vector<std::vector<cv::Point2f> > &corners,
        std::vector<cv::Vec3d> &rotationVectors, std::vector<cv::Vec3d> &translationVectors);

// Use the 3d position of the Camera with respect to the Aruco Marker
// and calculate its position in the world coordinates
void calculatePositions(cv::Vec3d &trCamAruco, cv::Mat &rotCamWorldSpace, 
        cv::Mat &trCamWorldSpace, cv::Mat &cameraMatrix, cv::Mat &trArucoWorld);

// Calculate Camera World Coordinates
void calculateWorldCamera(cv::Mat &cameraMatrix, cv::Mat &rotCamWorldSpace, cv::Mat &trCamWorldSpace, cv::Mat &trCamWorld,
        cv::Mat &markerPosition);

// --------------------------------------------------------------------------------------------------------------------
// FUNCTIONS FOR DRAWING ARROW
// --------------------------------------------------------------------------------------------------------------------

// Calculate the angle for the object
float calculateAngle(cv::Mat &cameraMatrix, cv::Mat &rotCamWorldSpace, cv::Mat &trCamWorldSpace, 
        cv::Mat &markerPosition);

// Insert arrow into image
void insertArrow(cv::Mat &sourceImage, cv::Mat &arrow, cv::Mat &arrowBgOr, int angle, cv::Mat &destImage);

// --------------------------------------------------------------------------------------------------------------------
// DEFINING A DATA STRUCTURE TO STORE THE ARUCO POSITIONS
// --------------------------------------------------------------------------------------------------------------------
class ObjectPosition{
public:
    int id;
    cv::Mat translation;
};

#endif //AT3DCV_UTILS_H
