//
// Created by Stefano Gasperini on 12/10/18.
//

#include <iostream>
#include <sstream>
#include <fstream>
#include <Eigen/Dense>
#include <unsupported/Eigen/EulerAngles>
#include <math.h>
using namespace std;

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
		}
		while (r == 0.0 || r > 1.0);
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

// usage: first argument (after program name) is the ground truth poses file,
// second is the file to which to write the compass simulations
int main(int argc, char **argv) {
	cout << "opening " << argv[1] << endl;
	ifstream source;
	source.open(string(argv[1]), ios_base::in);
	if (!source)  {
		cerr << "Can't open input file!\n";
	}else{
		ofstream compassFile;
		compassFile.open(string(argv[2]));
		for(string line; getline(source, line); ) {
			istringstream in(line);

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

			// Euler angles taken from the rotation matrix
			Eigen::Vector3d ea = rot_mat.eulerAngles(0,1,2);

			// prints euler angles in radians
			cout << ea.transpose() << "\t\t";

			ea[0] = ea[0] * 180 / M_PI;
			ea[1] = ea[1] * 180 / M_PI;
			ea[2] = ea[2] * 180 / M_PI;
			// prints euler angles in degrees
			cout << ea.transpose() << endl;

			// saves to file the time frame and a random pick from the normal distribution
			// having mean the original angle and standard deviation 5
			// this compass will not be super accurate, rather random around the true value
			compassFile << tframe << " " << rand_normal(ea[0], 5) << endl;

			// prints some examples from the distribution
			for (int i = 0; i < 15; ++i) {
				cout << rand_normal(ea[0],5) << " ";
			}
			cout << "\n" << endl;

			//Eigen::AngleAxisd x_ang = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX());
			//cout << x_ang.angle() << endl;
		}
		compassFile.close();
		cout << "\nWrote all compass simulated data to " << string(argv[2]) << endl;
	}
}