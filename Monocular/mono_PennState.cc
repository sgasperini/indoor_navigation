/**
 * adapted by Stefano Gasperini
 * originally from ORB_SLAM2 <https://github.com/raulmur/ORB_SLAM2>
 */


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<unistd.h>
#include<opencv2/core/core.hpp>
#include<System.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include<utils/Utils.cpp>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
				vector<double> &vTimestamps);

void print_to_file_for_gt(double tframe, const cv::Mat &transl, ofstream &fileP, Eigen::Matrix<double, 3, 3> &rot_mat_e);

Eigen::Matrix<double, 3, 3> buildPoseWithCompass(Eigen::Matrix<double, 3, 3> rotation, double compass_reading,
												 double compass_offset);

double get_compass_angle_from_matrix(const Eigen::Matrix<double, 3, 3> &rotation);

double get_compass_angle_from_matrix(const cv::Mat &rotation);

int main(int argc, char **argv){
	if(argc != 4){
		cerr << endl << "Usage: ./mono_PennState path_to_vocabulary path_to_settings path_to_sequence" << endl;
		return 1;
	}

	// Retrieve paths to images
	vector<string> vstrImageFilenames;
	vector<double> vTimestamps;
	string strFile = string(argv[3])+"/rgb.txt";
	LoadImages(strFile, vstrImageFilenames, vTimestamps);

	int nImages = vstrImageFilenames.size();

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

	// Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;
	cout << "Images in the sequence: " << nImages << endl << endl;

	string compass_file_name = "/Users/Steve/Documents/TUM/Advanced3DComputerVision/PennState_dataset/data/additional_data/af/compass_simulation.txt";
	ifstream source_compass;
	source_compass.open(compass_file_name, ios_base::in);
	if (!source_compass)  {
		cerr << "Can't open compass input file: " << compass_file_name << endl;
		return -1;
	}

	//Initializing the Kalman Filter
	cv::KalmanFilter KF;
	int nStates = 18;
	int nMeasurements = 6;
	int nInputs = 0;
	double dt = 0.1;
	initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);
	cv::Mat measurements(nMeasurements, 1, CV_64F);
	measurements.setTo(cv::Scalar(0));

	cv::Mat translation_estimated(3, 1, CV_64F);
	cv::Mat rotation_estimated(3, 3, CV_64F);

	// Main loop
	int main_error = 0;
	std::thread runthread([&]() {  // Start in new thread
		cv::Mat im;
		cv::Mat frame_vis;

		/*ofstream fileTUM;
		fileTUM.open(string(argv[3]) + "/CameraTrajectoryTUM.txt");
		ofstream fileP;
		fileP.open(string(argv[3]) + "/CameraTrajectoryP.txt");*/
		/*std::vector<Eigen::Quaterniond> quaternions;
		std::vector<cv::Mat> translations;*/
		bool failed_all_so_far = true;
		bool just_failed = false;
		int count_failed = 0;
		cv::Mat cumul_rot = cv::Mat::eye(3, 3, CV_64F);
		Eigen::Matrix<double, 3, 3> cumul_rotation_e;
		cv2eigen(cumul_rot, cumul_rotation_e);
		Eigen::Matrix<double, 3, 3> last_rotation;
		cv::Mat cumul_translation = cv::Mat(1, 3, CV_32F, {0, 0, 0});
		cv::Mat last_translation;

		double compass_offset;
		double compass_reset_offset = 0;
		int angle;
		int prev_angle;

		for (int ni = 0; ni < nImages; ni++) {
			im = cv::imread(string(argv[3]) + "/" + vstrImageFilenames[ni], CV_LOAD_IMAGE_UNCHANGED);
			frame_vis = im.clone();

			double tframe = vTimestamps[ni];

			double compass_reading;
			bool compass_available = false;

			string line;
			if (getline(source_compass, line)) {
				istringstream in(line);
				double tframe_compass;
				in >> tframe_compass;
				//cout << tframe_compass << " " << tframe;
				//if(tframe == tframe_compass){
				in >> compass_reading;
				compass_available = true;
				//}
			}

			if (im.empty()) {
				cerr << endl << "Failed to load image at: "
					 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
				main_error = 1;
				return -2;
			}

/*#ifdef COMPILEDWITHC11
			std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
			std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif*/

			// Pass the image to the SLAM system
			cv::Mat currentPose = SLAM.TrackMonocular(im, tframe);

/*#ifdef COMPILEDWITHC11
			std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
			std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
			double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
			vTimesTrack[ni] = ttrack;
			// Wait to load the next frame
			double T = 0;
			if (ni < nImages - 1)
				T = vTimestamps[ni + 1] - tframe;
			else if (ni > 0)
				T = tframe - vTimestamps[ni - 1];
			if (ttrack < T)
				usleep((T - ttrack) * 1e6);*/

			double angle_est;

			if (currentPose.empty()) {
				cout << "frame " << ni << " - fail" << endl;
				SLAM.SetLost(true);
				count_failed++;
				if (!failed_all_so_far) {
					//quaternions.push_back(quaternions[ni - 1]);
					//translations.push_back(translations[ni - 1]);
					if (just_failed or count_failed > 8){
						cout << (just_failed ? "\nForcing reset because of just_failed" :
								 "\nForcing reset because of count_failed") << endl;
						SLAM.Reset();
						just_failed = false;
						count_failed = 0;
						cumul_rotation_e = last_rotation;
						last_translation.copyTo(cumul_translation);
					}
					if (compass_available){
						Eigen::Matrix<double, 3, 3> compass_rotation = buildPoseWithCompass(last_rotation,
																							compass_reading,
																							compass_offset);
						cv::Mat_<double> rot_mat_cv(3, 3);
						cv::eigen2cv(compass_rotation, rot_mat_cv);
						fillMeasurementsPennState(measurements, last_translation, rot_mat_cv);

						updateKalmanFilter(KF, measurements, translation_estimated, rotation_estimated);
						//updateKalmanFilter(KF, measurements, translation_estimated, rotation_estimated);
						angle_est = get_compass_angle_from_matrix(rotation_estimated);
						compass_reset_offset = compass_offset - compass_reading;
					}
					//print_to_file_for_gt(tframe, last_translation, fileP, last_rotation);
				} else {
					angle_est = -180;
					/*Eigen::Quaterniond tmp(0, 0, 0, 0);
					quaternions.push_back(tmp);
					translations.push_back(cv::Mat(1, 3, CV_32F, {0, 0, 0}));*/
				}
			} else {
				cout << "frame " << ni << " - success" /*<< " " << compass_offset*/ << endl;
				SLAM.SetLost(false);
				just_failed = true;
				count_failed = 0;

				cv::Mat rot_mat = currentPose(cv::Range(0, 3), cv::Range(0, 3));

				Eigen::Matrix<double, 3, 3> rot_mat_e;
				cv2eigen(rot_mat, rot_mat_e);

				double tmp = get_compass_angle_from_matrix(rot_mat_e);

				rot_mat_e = rot_mat_e * cumul_rotation_e;
				Eigen::Quaterniond q(rot_mat_e);
				q.normalize();
				//cout << "\t\t\t\t" << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << endl;

				last_rotation = Eigen::Matrix<double, 3, 3>(rot_mat_e);

				cv::Mat transl = currentPose.rowRange(0, 3).col(3);
				cv::transpose(transl, transl);
				transl = transl + cumul_translation;
				//cout << "\t\t\t\t" << transl << endl;
				transl.copyTo(last_translation);

				/*quaternions.push_back(q);
				translations.push_back(transl);*/

				cv::Mat_<double> rot_mat_cv(3, 3);
				/*cv::eigen2cv(rot_mat_e, rot_mat_cv);
				fillMeasurementsPennState(measurements, transl, rot_mat_cv);

				updateKalmanFilter(KF, measurements, translation_estimated, rotation_estimated);*/

				if (failed_all_so_far) {
					if (compass_available){
						compass_offset = double(compass_reading - get_compass_angle_from_matrix(last_rotation));
						SLAM.DrawArrow(true);
					}
					failed_all_so_far = false;
				}

				if (compass_available) {
					//cv::cv2eigen(rotation_estimated, rot_mat_e);  // remove for compass only
					/*Eigen::Matrix<double, 3, 3> compass_rotation = buildPoseWithCompass(rot_mat_e, compass_reading,
																						compass_offset);
//					cv::Mat_<double> rot_mat_cv(3, 3);
					cv::eigen2cv(compass_rotation, rot_mat_cv);
					fillMeasurementsPennState(measurements, transl, rot_mat_cv);

					// update the Kalman filter with good measurements
					updateKalmanFilter(KF, measurements, translation_estimated, rotation_estimated);*/
//					angle_est = get_compass_angle_from_matrix(rotation_estimated);
					angle_est = -fmod(tmp + compass_reset_offset, 360);
					if(abs(-compass_offset+compass_reading - angle_est) < 15){
						cout << "\t\t\t\t\tgood angle" << endl;
						/*if((int(angle_est) ^ int(compass_offset-compass_reading)) < 0){
							// opposite signs
							if(!((angle_est < 0 and ((10 < angle_est+360 and angle_est+360 < 190)
												 or (-10 < angle_est+10 and angle_est+10 < 10)))
								 or (angle_est > 0 and ((170 < angle_est and angle_est < 190)
												   or (-10 < angle_est-10 and angle_est-10 < 10))))) {
								cout << "\t\t\t\t\treversed" << endl;
								angle_est = -angle_est;
							}
						}*/
						//angle_est = -angle_est;
					}else{
						cout << "\t\t\t\t\tbad angle" << endl;
						angle_est = -(compass_offset-compass_reading);
					}
				}

				// to print to file and compare against the PennCOSYVIO gt
				//print_to_file_for_gt(tframe, transl, fileP, rot_mat_e);
			}

			if(!failed_all_so_far){
				if(currentPose.empty()){
					angle = -int(180 - (compass_offset-compass_reading)); // if compass only
				}else{
					angle = int(180 - (angle_est));
				}
				/*if(abs(angle - prev_angle) > 20){
					angle = prev_angle;
				}else{
					prev_angle = angle;
				}*/
			}else{
				angle = 180;
				prev_angle = -angle;
			}

			cout << "\t\t\t" << int(compass_reading) << " " << int(angle_est) << " " << angle << endl;

			// takes counter-clockwise
			// -angle if compass only
			SLAM.SetAngle(angle);

			//cout << translations[ni] << endl;

			/*char aa[50];
			sprintf(aa, "%.2f", tframe);
			fileTUM << aa << "\t\t\t";
			sprintf(aa, "%.7f", quaternions[ni].w());
			fileTUM << aa << " ";
			sprintf(aa, "%.7f", quaternions[ni].x());
			fileTUM << aa << " ";
			sprintf(aa, "%.7f", quaternions[ni].y());
			fileTUM << aa << " ";
			sprintf(aa, "%.7f", quaternions[ni].z());*/
			/*fileTUM << aa << " ";
			sprintf(aa, "%.7f", translations[ni].at<double>(0));
			fileTUM << aa << " ";
			sprintf(aa, "%.7f", translations[ni].at<double>(1));
			fileTUM << aa << " ";
			sprintf(aa, "%.7f", translations[ni].at<double>(2));*/
			//fileTUM << aa << endl;
		}

		/*fileTUM.close();
		fileP.close();*/
	}); // End the thread

	// Start the visualization thread
	SLAM.StartViewer();

	cout << "Viewer started, waiting for thread." << endl;
	runthread.join();
	if (main_error != 0)
		return main_error;
	cout << "Tracking thread joined..." << endl;

	// Stop all threads
	SLAM.Shutdown();
	cout << "System Shutdown" << endl;

	// Tracking time statistics
	sort(vTimesTrack.begin(),vTimesTrack.end());
	float totaltime = 0;
	for(int ni=0; ni<nImages; ni++){
		totaltime+=vTimesTrack[ni];
	}
	cout << "-------" << endl << endl;
	cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
	cout << "mean tracking time: " << totaltime/nImages << endl;

	// Save camera trajectory
	// SLAM.SaveTrajectoryTUM(strFile + "/FrameTrajectory.txt");
	SLAM.SaveKeyFrameTrajectoryTUM(string(argv[3]) + "/KeyFrameTrajectory.txt");

	return 0;
}

Eigen::Matrix<double, 3, 3> buildPoseWithCompass(Eigen::Matrix<double, 3, 3> rotation, double compass_reading,
												 double compass_offset) {
	double angle_from_rotation = get_compass_angle_from_matrix(rotation);
	angle_from_rotation = angle_from_rotation + compass_offset;

	cout << "\t" << int(angle_from_rotation) << "\t" << int(compass_reading) << endl;

	Eigen::Matrix<double, 3, 3> compass_rotation;
	double a = compass_reading - compass_offset;
	compass_rotation << cos(a),  0, sin(a),
						0, 		 1, 0,
						-sin(a), 0, cos(a);

	return compass_rotation;
}

double get_compass_angle_from_matrix(const Eigen::Matrix<double, 3, 3> &rotation) {
	//Eigen::Vector3d ea = rotation.eulerAngles(0, 1, 2);
	// prints euler angles in radians
	// cout << ea.transpose() << "\t\t";
	//ea[1] = ea[1] * 180 / M_PI;

	Eigen::RowVector3d x_of_R;
	x_of_R << rotation(0,0), rotation(1,0), rotation(2,0);
	/*Eigen::RowVector3d y_of_id;
	y_of_id << 0, 1, 0;*/
	double cos_a = rotation(0,0) / x_of_R.norm();
	double alpha = acos(cos_a);
	alpha = alpha * 180 / M_PI;

//	return ea[1];
	return alpha;
}

double get_compass_angle_from_matrix(const cv::Mat &rotation) {
	Eigen::Matrix<double, 3, 3> rotation_eigen;
	cv::cv2eigen(rotation, rotation_eigen);
	return get_compass_angle_from_matrix(rotation_eigen);
}

void print_to_file_for_gt(double tframe, const cv::Mat &transl, ofstream &fileP, Eigen::Matrix<double, 3, 3> &rot_mat_e) {
	Eigen::Matrix<double,3,1> transl_e;
	cv2eigen(transl, transl_e);
	fileP << tframe << " ";
	fileP << rot_mat_e(0,0) << " ";
	fileP << rot_mat_e(0,1) << " ";
	fileP << rot_mat_e(0,2) << " ";
	fileP << transl_e(0) << " ";
	fileP << rot_mat_e(1,0) << " ";
	fileP << rot_mat_e(1,1) << " ";
	fileP << rot_mat_e(1,2) << " ";
	fileP << transl_e(1) << " ";
	fileP << rot_mat_e(2,0) << " ";
	fileP << rot_mat_e(2,1) << " ";
	fileP << rot_mat_e(2,2) << " ";
	fileP << transl_e(2) << endl;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps) {
	ifstream f;
	f.open(strFile.c_str());

	// skip first three lines
	string s0;
	getline(f,s0);
	getline(f,s0);
	getline(f,s0);

	while(!f.eof()){
		string s;
		getline(f,s);
		if(!s.empty()){
			stringstream ss;
			ss << s;
			double t;
			string sRGB;
			ss >> t;
			vTimestamps.push_back(t);
			ss >> sRGB;
			vstrImageFilenames.push_back(sRGB);
		}
	}
}
