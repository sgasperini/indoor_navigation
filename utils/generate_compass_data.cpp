//
// Created by Stefano Gasperini on 12/10/18.
//

#include <iostream>
#include <sstream>
#include <fstream>
#include <Eigen/Dense>
#include <unsupported/Eigen/EulerAngles>
#include <math.h>
#include <System.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <typeinfo>
#include <opencv/cv.hpp>

using namespace std;

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

double rand_normal(double mean, double stddev) {
	//Box muller method
	static double n2 = 0.0;
	static int n2_cached = 0;
	if (!n2_cached){
		double x, y, r;
		do {
			x = 2.0*rand()/RAND_MAX - 1;
			y = 2.0*rand()/RAND_MAX - 1;
			r = x*x + y*y;
		} while (r == 0.0 || r > 1.0);
		{
			double d = sqrt(-2.0*log(r)/r);
			double n1 = x*d;
			n2 = y*d;
			double result = n1*stddev + mean;
			n2_cached = 1;
			return result;
		}
	}else{
		n2_cached = 0;
		return n2*stddev + mean;
	}
}

std::string FloatToString ( float Number ) {
	std::ostringstream ss;
	ss << int(Number);
	return ss.str();
}

// usage: first argument (after program name) is the ground truth poses file,
// second is the file to which to write the compass simulations
int main(int argc, char **argv) {

	bool visualize_angles = true;

	cout << "opening " << argv[1] << endl;
	ifstream source;
	source.open(string(argv[1]), ios_base::in);
	if (!source)  {
		cerr << "Can't open input file!\n";
	}else{

		cv::Mat arrow = cv::imread("small_arrow.png", CV_LOAD_IMAGE_UNCHANGED);

		vector<string> vstrImageFilenames;
		cv::Mat cameraMatrix, distCoeffs;
		if(visualize_angles) {
			// Retrieve paths to images
			vector<double> vTimestamps;
			string strFile = "/Users/Steve/Documents/TUM/Advanced3DComputerVision/PennState_dataset/data/visensor/af/rgb.txt";
			LoadImages(strFile, vstrImageFilenames, vTimestamps);

			float data[3][3] = { {4.457995128417765e+02,0.000000,3.715033626637895e+02},{0.000000,4.451497175947849e+02,2.373274704309797e+02},{0.000000, 0.000000, 1.000000} };
			float datasecond[5][1] = { {-0.03671},{0.05260},{0.0}, {0.0}, {0.0} };
			cameraMatrix = cv::Mat(3, 3, CV_32FC1, &data);
			distCoeffs = cv::Mat(5, 1, CV_32FC1, &datasecond);
		}

		ofstream compassFile;
		compassFile.open(string(argv[2]));

		int ni_img = 0;
		int ni_gt = 0;
		cv::Mat im;
		cv::Mat frame_vis;

		Eigen::Matrix3d r_from_viL_to_c2;
		r_from_viL_to_c2 << 0.9991687124, -0.0059124048, -0.0403351908,
							0.0050322682, 0.9997477793, -0.0218873038,
							0.0404544240, 0.0216661317, 0.9989464542;

		for(string line; getline(source, line); ) {
			istringstream in(line);
			ni_gt++;
			if(ni_gt % 3 == 0){
				continue;
			}

			if(visualize_angles) {
				im = cv::imread(
						"/Users/Steve/Documents/TUM/Advanced3DComputerVision/PennState_dataset/data/visensor/af/" +
						vstrImageFilenames[ni_img], CV_LOAD_IMAGE_UNCHANGED);
				ni_img++;

//				if(ni_img < 770){
//					cout << ni_img << endl;
//					continue;
//				}
				frame_vis = im.clone();    // refresh visualisation frame
			}

			// for the file format check https://daniilidis-group.github.io/penncosyvio/file_format/

			double tframe;
			in >> tframe;

			double r11,r12,r13;
			in >> r11 >> r12 >> r13;
			double t1,t2,t3;
			in >> t1;
			double r21,r22,r23;
			in >> r21 >> r22 >> r23;
			in >> t2;
			double r31,r32,r33;
			in >> r31 >> r32 >> r33;

			// rotation matrix
			Eigen::Matrix3d rot_mat;
			rot_mat << r11,r12,r13,
				 r21,r22,r23,
				 r31,r32,r33;

			// conversion to camera coord system
			rot_mat = r_from_viL_to_c2.transpose() * rot_mat;

			Eigen::RowVector3d x_of_R;
			x_of_R << r11, r21, r31;
			/*Eigen::RowVector3d y_of_id;
			y_of_id << 0, 1, 0;*/
			double cos_a = r11 / x_of_R.norm();
			double alpha = acos(cos_a);
			alpha = alpha * 180 / M_PI;

			/*Eigen::RowVector3d y_of_R;
			y_of_R << r12, r22, r32;
			Eigen::RowVector3d y_of_id;
			y_of_id << 0, 1, 0;
			double cos_b = r22 / y_of_R.norm();
			double beta = acos(cos_b);
			beta = beta * 180 / M_PI;

			Eigen::RowVector3d z_of_R;
			z_of_R << r13, r23, r33;
			Eigen::RowVector3d y_of_id;
			y_of_id << 0, 1, 0;
			double cos_c = r33 / z_of_R.norm();
			double gamma = acos(cos_c);
			gamma = gamma * 180 / M_PI;*/

			// Euler angles taken from the rotation matrix
			Eigen::Vector3d ea = rot_mat.eulerAngles(0,1,2);

			// prints euler angles in radians
			//cout << ea.transpose() << "\t\t";
			cout << ni_img << "\t\t";

			//ea[0] = ea[0] * 180 / M_PI;
			ea[1] = ea[1] * 180 / M_PI;
			//ea[2] = ea[2] * 180 / M_PI;
			// prints euler angles in degrees
			//std::string str = "angles:   " + FloatToString(ea[0]) + "   " + FloatToString(ea[1]) + "   " + FloatToString(ea[2]);
			std::string str = "angle:   " + FloatToString(alpha); // + "   " + FloatToString(beta) + "   " + FloatToString(gamma);
			std::string str2 = "         " + /*FloatToString(ea[0]) + "   " +*/ FloatToString(ea[1]);// + "   " + FloatToString(ea[2]);
			cout << ea[1] /*.transpose()*/ << endl;

			cout << "\t\t" << alpha /*<< "\t" << beta << "\t" << gamma*/ << endl;

			if(visualize_angles){ // and ni_img > 770){
				int fontFace = cv::FONT_ITALIC;
				double fontScale = 0.75;
				int thickness_font = 2;
				cv::Scalar red(0, 0, 255);
				cv::putText(frame_vis, str, cv::Point(10,420), fontFace, fontScale, red, thickness_font, 8);
				cv::putText(frame_vis, str2, cv::Point(10,445), fontFace, fontScale, red, thickness_font, 8);
				cv::Vec3d rvec;
				cv::Mat_<double> rot_mat_cv(3,3);
				cv::eigen2cv(rot_mat, rot_mat_cv);
				//cout << rot_mat_cv << endl;
				cv::Rodrigues(rot_mat_cv, rvec);
				//cout << rvec << endl;
				cv::Vec3d ttmp;
				ttmp[0] = t1;
				ttmp[1] = t2;
				ttmp[2] = t3;

				cv::Mat arrow_bg = cv::imread("s_b_arrow_bg.png", CV_LOAD_IMAGE_UNCHANGED);
				cvtColor(arrow_bg, arrow_bg, 1);

				cv::Point2f pc(arrow_bg.cols/2., arrow_bg.rows/2.);
				cv::Mat r = cv::getRotationMatrix2D(pc, ea[1], 1.0);
				cv::warpAffine(arrow_bg, arrow_bg, r, arrow_bg.size());

				cv::Rect roi(584,128, 752, 480);
				arrow_bg = arrow_bg(roi);

				//double a = 0.5;
//				cv::Mat overlay;
//				overlay = arrow_bg.clone();
//				cv::rectangle(arrow_bg, cv::Point(1, 1), cv::Point(1, 1), red, -1);
//				cv::addWeighted(frame_vis, a, arrow, 1 - a, 0, frame_vis);
//				cv::addWeighted(frame_vis, alpha, arrow_bg, 1 - alpha, 0, frame_vis);

				// "brute-force"
				for(int y=0;y<arrow_bg.rows;y++) {
					for (int x = 0; x < arrow_bg.cols; x++) {
						// get pixel
						cv::Vec3b color = arrow_bg.at<cv::Vec3b>(cv::Point(x, y));
						if(color.val[0] + color.val[1] + color.val[2] > 0){
							frame_vis.at<cv::Vec3b>(cv::Point(x, y)) = color;
						}
					}
				}

				cv::imshow("angles", frame_vis);
				cv::waitKey(0);
			}



			// saves to file the time frame and a random pick from the normal distribution
			// having mean the original angle and standard deviation 5
			// this way the compass will not be super accurate, rather random around the true value
			compassFile << tframe << " " << rand_normal(ea[1], 5) << endl;

			// prints some examples from the distribution
			for (int i = 0; i < 15; ++i) {
				cout << rand_normal(ea[1],5) << " ";
			}
			cout << "\n" << endl;

			//Eigen::AngleAxisd x_ang = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX());
			//cout << x_ang.angle() << endl;
		}
		compassFile.close();
		cout << "\nWrote all compass simulated data to " << string(argv[2]) << endl;
	}
}
