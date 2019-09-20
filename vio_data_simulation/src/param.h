//
// Created by hyj on 17-6-22.
//

#ifndef IMUSIM_PARAM_H
#define IMUSIM_PARAM_H

#include <eigen3/Eigen/Core>

class Param{

public:

    Param();

    // time
    int imu_frequency = 200;
    int cam_frequency = 30;
    double imu_timestep = 1./imu_frequency;
    double cam_timestep = 1./cam_frequency;
    double t_start = 0.;
    double t_end = 20;  //  20 s

    // // bias
    // double gyro_bias_sigma = 1.0e-5;
    // double acc_bias_sigma = 0.0001;

    // // noise
    // double gyro_noise_sigma = 0.015;    // rad/s
    // double acc_noise_sigma = 0.019;      //　m/(s^2)

    // bias
    // double gyro_bias_sigma = 2.0e-6 /14.14;
    // double acc_bias_sigma = 0.00004 /14.14;

    // // noise
    // double gyro_noise_sigma = 0.004 /14.14;    // rad/s
    // double acc_noise_sigma = 0.008 /14.14;      //　m/(s^2)


    // double pixel_noise = 1;              // 1 pixel noise

    // double gyro_bias_sigma = 0.0;
    // double acc_bias_sigma = 0.0;

    // // noise
    // double gyro_noise_sigma = 0.0;  // rad/s
    // double acc_noise_sigma = 0.0;      //　m/(s^2)
    double gyro_bias_sigma = 2.0e-6 /14.14;
    double acc_bias_sigma = 0.00003185/14.14;

    // noise
    double gyro_noise_sigma = 0.00009369/14.14;    // rad/s
    double acc_noise_sigma = 0.00055935/14.14;      //　m/(s^2)


    double pixel_noise = 0.0;              // 1 pixel noise


    // cam f
    double fx = 460;
    double fy = 460;
    double cx = 320;
    double cy = 240;
    double image_w = 640;
    double image_h = 480;


    // 外参数
    Eigen::Matrix3d R_bc;   // cam to body
    Eigen::Vector3d t_bc;     // cam to body

};


#endif //IMUSIM_PARAM_H
