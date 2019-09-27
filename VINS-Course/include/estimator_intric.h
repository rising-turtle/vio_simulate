/*

	Sep. 25 2019, He Zhang, hzhang8@vcu.edu 

	new estimator taking camera intrinsic 

*/

#pragma once 
#include "estimator.h"


class EstimatorIntric : public Estimator{

public:
	EstimatorIntric(); 

    virtual ~EstimatorIntric();

   // internal
    void clearState();
    VecX getResult();
    void slideWindow();
    void solveOdometry();

    void problemSolve();
    void MargOldFrame();
    void MargNewFrame();

    void vector2double();
    void double2vector();

	Vector3d CamIncs[(WINDOW_SIZE + 1)]; 
    double para_CamIntrics[WINDOW_SIZE + 1][3]; // camera intrinsic: f, cx, cy
    
};