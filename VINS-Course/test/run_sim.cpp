
/*
	Sep. 25 2019, He Zhang, hzhang8@vcu.edu 

	run simulated data 

*/

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include "System.h"
#include "get_imu.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 5; 

// string sData_path = "/home/hzhang8/work/github/vio_simulate/vio_data_simulation/bin/";

string sData_path = "../../vio_data_simulation/bin/";

string sConfig_path = "../config/";

std::shared_ptr<System> pSystem;

IMUDataSet imu_set;

void PubImuData()
{
	// std::string sImu_line;
	double dStampNSec = 0.0;
	Vector3d vAcc;
	Vector3d vGyr;

	//pose
	Vector4d q;
	Vector3d t;

	int cnt = 0; 
	IMUData d; 

	// while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
	while(imu_set.getIMUAt(cnt++, d))
	{
		// ssImuData >> dStampNSec >> q.w() >> q.x() >> q.y() >> q.z() >>
        //        t.x() >> t.y() >> t.x() >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
        dStampNSec = d.mtimestamp; 
        q.w() = d.mq[0]; q.x() = d.mq[1]; q.y() = d.mq[2]; q.z() = d.mq[3]; 
        t.x() = d.mt[0]; t.y() = d.mt[1]; t.z() = d.mt[2]; 
        vGyr.x() = d.mgyro[0]; vGyr.y() = d.mgyro[1]; vGyr.z() = d.mgyro[2]; 
        vAcc.x() = d.macc[0]; vAcc.y() = d.macc[1]; vAcc.z() = d.macc[2]; 

		// cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
		pSystem->PubImuData(dStampNSec, vGyr, vAcc); //Modified
		usleep(1000*5*3); // ms 200 hz about 5ms  
	}
}

void compute_delta_intrinsic(double timestamp, double& df, double& dcx, double& dcy)
{
	df = dcx = dcy = 0; 
	IMUData d; 
	double kf, kcx, kcy; 
	kf = 0.3; kcx = kcy= 3.; 
	if(imu_set.getIMUData(timestamp, d)){
		Vector3d vAcc, wAcc, wAcc_nog, cAcc; // in body 
		Quaterniond q;
		q.w() = d.mq[0]; q.x() = d.mq[1]; q.y() = d.mq[2]; q.z() = d.mq[3]; 
		vAcc.x() = d.macc[0]; vAcc.y() = d.macc[1]; vAcc.z() = d.macc[2]; 
		wAcc = q * vAcc; // qwb * acc_b 
		wAcc_nog = wAcc; 
		wAcc_nog.z() += -9.81; 
		auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
		cAcc = RIC[0].inverse() * q.toRotationMatrix().inverse() * wAcc_nog; // Rcb * Rbw * a_w 
		// cout <<" run_sim: at "<<timestamp<<" vAcc: "<<vAcc.transpose()<<" wAcc: "<<wAcc.transpose()<<
		// " wAcc_nog: "<<wAcc_nog.transpose()<<" cAcc: "<<cAcc<<endl;

		cout <<"run_sim: gt at "<<timestamp<<" pose: "<<d.mt[0]<<", "<<d.mt[1]<<", "<<d.mt[2]<<" euler: "<<euler.transpose()<<endl;

		df = kf * cAcc.z(); 
		dcx = kcx * cAcc.x(); 
		dcy = kcy * cAcc.y(); 
		// cout <<" delta intrinsic are: df = "<<df<<" dcx = "<<dcx<<" dcy = "<<dcy<<endl;
	}
	return ; 
}


void PubImageData()
{
	// string sImage_file = sConfig_path + "MH_05_cam0.txt";
	string sImage_file = sData_path + "Features/";
	string sTimestamp_file = sImage_file + "Timestamp.txt";

	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsTimestamp; //modified
	fsTimestamp.open(sTimestamp_file.c_str());//Added

	if (!fsTimestamp.is_open())
	{
		cerr << "Failed to open Timestamp file! " << sTimestamp_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	double dStampNSec_pre = 0;
	std::vector<Vector2d> feature_positions_pre;
	string sImgFileName;

	// ground truth 
	double fx = FOCAL_LENGTH;  double fy = FOCAL_LENGTH;// 460.
	double cx = CX; // 320.0; 
	double cy = CY; // 240.0;
	
	double dfx, dcx, dcy; 
	double tf = fx; double tcx = cx; double tcy = cy; 

	int cnt = 0; // count frame number 

	// cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
	while (std::getline(fsTimestamp, sImage_line) && !sImage_line.empty())
	{
		std::istringstream ssImuData(sImage_line);
		ssImuData >> dStampNSec >> sImgFileName;
		++cnt; 
		
		string imagePath = sData_path + sImgFileName;
		
		ifstream fsFeatures;
 		fsFeatures.open(imagePath.c_str());
		
		if(!fsFeatures.is_open())
		{
			cerr << "Failed to open TimeStep file! " << sTimestamp_file << endl;
			return;
		}


		std::vector<Vector2d> feature_positions_un,feature_positions; 
		//For HW2, it gets the position on the normalized plane
		std::vector<Vector2d> feature_velocities;
		std::vector<int> ids;
		int count = 0;
		double dt = dStampNSec - dStampNSec_pre;


		if(cnt > 20){ // skip 12 frames for initialization 
				
			compute_delta_intrinsic(dStampNSec, dfx, dcx, dcy);
			tf += dfx * dt; 
			tcx += dcx * dt; 
			tcy += dcy * dt;

			cout <<"run_sim: gt at "<< dStampNSec<<" f: "<<tf<<" cx: "<< tcx<<" cy: "<<tcy<<endl;

		} 

		// cout <<"run_sim: current intrinsic: fx= "<<tf<<" cx: "<<tcx<<" cy: "<<tcy<<endl; 
		std::string sFeature_line;
		while (std::getline(fsFeatures, sFeature_line) && !sFeature_line.empty())
		{
			Vector4d landmark;
			Vector2d feature_position, feature_position_un;
			Vector2d feature_velocity;

			std::istringstream ssFeatureData(sFeature_line);
			ssFeatureData >> landmark.x() >> landmark.y() >> landmark.z() >> landmark.w() >> feature_position.x() >> feature_position.y();

			// feature_position.x() = feature_position.x()*460.0 + 320.0;
			// feature_position.y() = feature_position.y()*460.0 + 240.0;
			
			// true measurement, since tf, tcx, tcy are true intrinsic param
			feature_position.x() = feature_position.x()*tf + tcx; 
			feature_position.y() = feature_position.y()*tf + tcy;
			feature_positions.push_back(feature_position);

			// reproject to the wrong ray direction 
			feature_position_un.x() = (feature_position.x()-cx)/fx;
			feature_position_un.y() = (feature_position.y()-cy)/fy;
			feature_positions_un.push_back(feature_position_un);

			//This dataset has aligned the feature points
			//Calculate the velocity //cur - pre; //in pixel coordinate
			if(dt==0){//initialization
				feature_velocity.x() = 0;
				feature_velocity.y() = 0;	
			}
			else{
				feature_velocity.x() = (feature_position.x() - feature_positions_pre[count].x())/(dt); 
				feature_velocity.y() = (feature_position.y() - feature_positions_pre[count].y())/(dt);
			}
			feature_velocities.push_back(feature_velocity);
			
			ids.push_back(count);	

			count++;
		}
		
		feature_positions_pre = feature_positions;
		dStampNSec_pre = dStampNSec;

		fsFeatures.close();
		
		pSystem->PubFeatureData(dStampNSec , ids, /*landmarks,*/feature_positions_un, feature_positions, feature_velocities);

		usleep(1000*10*5*nDelayTimes); // 30 hz, 33 ms 
	}
	fsTimestamp.close();
}


int main(int argc, char **argv)
{
	// if(argc != 3)
	// {
	// 	cerr << "./run_euroc PATH_TO_FOLDER/MH-05/mav0 PATH_TO_CONFIG/config \n" 
	// 		<< "For example: ./run_euroc /home/stevencui/dataset/EuRoC/MH-05/mav0/ ../config/"<< endl;
	// 	return -1;
	// }
	// sData_path = argv[1];
	// sConfig_path = argv[2];

	// read imu first 
	string imu_file = sData_path + "imu_pose_noise.txt";
	if(!imu_set.readIMUFile(imu_file)){
		cerr <<" failed to read imu file "<<imu_file<<endl; 
		return -1;
	}

	pSystem.reset(new System(sConfig_path));
	
	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);
		
	// sleep(5);
	std::thread thd_PubImuData(PubImuData);

	std::thread thd_PubImageData(PubImageData);

#ifdef __linux__	
	std::thread thd_Draw(&System::Draw, pSystem);
#elif __APPLE__
	// DrawIMGandGLinMainThrd();
#endif

	thd_PubImuData.join();
	thd_PubImageData.join();

	// thd_BackEnd.join();
#ifdef __linux__	
	thd_Draw.join();
#endif

	cout << "main end... see you ..." << endl;
	return 0;
}
