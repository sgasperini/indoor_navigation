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

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
				vector<double> &vTimestamps);

int main(int argc, char **argv)
{
	if(argc != 4)
	{
		cerr << endl << "Usage: ./mono_aruco path_to_vocabulary path_to_settings path_to_sequence" << endl;
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

	// Main loop
	int main_error = 0;
	std::thread runthread([&]() {  // Start in new thread
		cv::Mat im;

		ofstream fileTUM;
		fileTUM.open(string(argv[3]) + "/CameraTrajectoryTUM.txt");
		//ofstream fileP;
		//fileP.open(string(argv[3]) + "/CameraTrajectoryP.txt");
		std::vector<Eigen::Quaterniond> quaternions;
		std::vector<cv::Mat> translations;
		bool failed_all_so_far = true;
		bool just_failed = false;
		int count_failed = 0;
		cv::Mat cumul_rot = cv::Mat::eye(3, 3, CV_64F);
		Eigen::Matrix<double,3,3> cumul_rotation_e;
		cv2eigen(cumul_rot, cumul_rotation_e);
		Eigen::Matrix<double,3,3> last_rotation;
		cv::Mat cumul_translation = cv::Mat(1, 3, CV_32F, {0,0,0});
		cv::Mat last_translation;

		for(int ni=0; ni<nImages; ni++)
		{
			// Read image from file
			im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
			double tframe = vTimestamps[ni];

			if(im.empty())
			{
				cerr << endl << "Failed to load image at: "
					 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
				main_error = 1;
				return;
			}

#ifdef COMPILEDWITHC11
			std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
			std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

			// Pass the image to the SLAM system
			cv::Mat currentPose = SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
			std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
			std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

			double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

			vTimesTrack[ni]=ttrack;

			// Wait to load the next frame
			double T=0;
			if(ni<nImages-1)
				T = vTimestamps[ni+1]-tframe;
			else if(ni>0)
				T = tframe-vTimestamps[ni-1];

			if(ttrack<T)
				usleep((T-ttrack)*1e6);

			if(currentPose.empty()){
				cout << "frame " << ni << " - fail" << endl;
				count_failed++;
				//cout << "pose failed" << endl;
				if(!failed_all_so_far){
					quaternions.push_back(quaternions[ni-1]);
					translations.push_back(translations[ni-1]);
					if(just_failed){ //} or count_failed > 20){
						cout << (just_failed ? "\nForcing reset because of just_failed" :
								 "\nForcing reset because of count_failed") << endl;
						SLAM.Reset();
						just_failed = false;
						count_failed = 0;
						cumul_rotation_e = last_rotation;
						last_translation.copyTo(cumul_translation);
					}
				}else{
					Eigen::Quaterniond tmp(0,0,0,0);
					quaternions.push_back(tmp);
					translations.push_back(cv::Mat(1, 3, CV_32F, {0,0,1}));
				}
			}else{
				cout << "frame " << ni << " - success" << endl;
				failed_all_so_far = false;
				just_failed = true;
				count_failed = 0;

				//cout << currentPose << endl;

				cv::Mat rot_mat = currentPose(cv::Range(0,3), cv::Range(0,3));
				//cout << rot_mat << endl;

				Eigen::Matrix<double,3,3> rot_mat_e;
				cv2eigen(rot_mat, rot_mat_e);

				rot_mat_e = rot_mat_e * cumul_rotation_e;
				Eigen::Quaterniond q(rot_mat_e);
				q.normalize();
				cout << "\t\t\t\t" << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << endl;

				last_rotation = Eigen::Matrix<double,3,3>(rot_mat_e);


				cv::Mat transl = currentPose.rowRange(0,3).col(3);
				cv::transpose(transl, transl);
				transl = transl + cumul_translation;
				cout << "\t\t\t\t" << transl << endl;
				transl.copyTo(last_translation);

				//cout << transl << endl;

				quaternions.push_back(q);
				translations.push_back(transl);
				//cout << transl.at<double>(0) << "" << endl;
			}
			//cout << translations[ni] << endl;

			char aa[50];
			sprintf(aa, "%.2f", tframe);
			fileTUM << aa << "\t\t\t";
			sprintf(aa, "%.7f", quaternions[ni].w());
			fileTUM << aa << " ";
			sprintf(aa, "%.7f", quaternions[ni].x());
			fileTUM << aa << " ";
			sprintf(aa, "%.7f", quaternions[ni].y());
			fileTUM << aa << " ";
			sprintf(aa, "%.7f", quaternions[ni].z());
			/*fileTUM << aa << " ";
			sprintf(aa, "%.7f", translations[ni].at<double>(0));
			fileTUM << aa << " ";
			sprintf(aa, "%.7f", translations[ni].at<double>(1));
			fileTUM << aa << " ";
			sprintf(aa, "%.7f", translations[ni].at<double>(2));*/
			fileTUM << aa << endl;
		}

		fileTUM.close();
		//fileP.close();
		// SLAM.Shutdown();
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
	for(int ni=0; ni<nImages; ni++)
	{
		totaltime+=vTimesTrack[ni];
	}
	cout << "-------" << endl << endl;
	cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
	cout << "mean tracking time: " << totaltime/nImages << endl;

	// Save camera trajectory
	// SLAM.SaveTrajectoryTUM(strFile + "/FrameTrajectory.txt");
	SLAM.SaveKeyFrameTrajectoryTUM(string(argv[3]) + "/KeyFrameTrajectory.txt");

	//SLAM.SaveTrajectoryTUM(string(argv[3]) + "/CameraTrajectory.txt");

	return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
	ifstream f;
	f.open(strFile.c_str());

	// skip first three lines
	string s0;
	getline(f,s0);
	getline(f,s0);
	getline(f,s0);

	while(!f.eof())
	{
		string s;
		getline(f,s);
		if(!s.empty())
		{
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
