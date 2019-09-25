/*

	Sep. 25 2019, He Zhang, hzhang8@vcu.edu 

	a simple interface to get access to imu data 

*/

#pragma once

#include <vector>
#include <fstream>
#include <iostream>
#include <string>

using namespace std; 

struct IMUData{

	// pose Tw_imu(body)
	double mq[4]; 
	double mt[3]; 

	double mtimestamp; // timestamp of this imu data 
	double mgyro[3]; // gx gy gz in IMU/body frame 
	double macc[3]; // ax ay az 
};

class IMUDataSet{

public:
	IMUDataSet(); 
	~IMUDataSet(); 

	bool readIMUFile(string f);
	bool getIMUData(double t, IMUData&); 
	bool getIMUAt(int index, IMUData&); 

	vector<IMUData> mv_imus;

};