/*

	Sep. 25 2019, He Zhang, hzhang8@vcu.edu 

	a simple interface to get access to imu data 

*/

#include "get_imu.h"
#include <sstream>
#include <cmath>

IMUDataSet::IMUDataSet(){}
IMUDataSet::~IMUDataSet(){}

bool IMUDataSet::readIMUFile(string file)
{
	ifstream fsImu;
	fsImu.open(file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "get_imu: Failed to open imu file! " << file << endl;
		return false;
	}

	std::string sImu_line;
	IMUData d; 

	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
	{
		std::istringstream ssImuData(sImu_line);

		// q[4] w x y z
		ssImuData >> d.mtimestamp >> d.mq[0] >> d.mq[1] >> d.mq[2] >> d.mq[3] >> 
			d.mt[0] >> d.mt[1] >> d.mt[2] >> d.mgyro[0]  >> d.mgyro[1] >> d.mgyro[2] >> 
			d.macc[0] >> d.macc[1] >> d.macc[2]; 

		mv_imus.push_back(d); 

		// usleep(1000*5); // ms 200 hz about 5ms  
	}
	fsImu.close();
	return mv_imus.size() > 0;
}

// find next whose timestamp is larger than t
bool IMUDataSet::getIMUData(double t, IMUData& imu)
{
	for(int i=0; i<mv_imus.size(); i++){
		double dt = mv_imus[i].mtimestamp - t; 

		if(fabs(dt) <= (0.0025))// 200 ms 5ms 
		{
			imu = mv_imus[i]; 
			return true; 
		}
	}
	return false;
}

bool IMUDataSet::getIMUAt(int index, IMUData& imu)
{
	if(index >= 0 && index < mv_imus.size()){
		imu = mv_imus[index]; 
		return true; 
	}
	return false; 
}