/*
	Sep. 26 2019, He Zhang, hzhang8@vcu.edu 

	test edge jacobians 
	
*/

#include <iostream>
#include "backend/backend.h"
#include "backend/edge.h"
#include "backend/edge_reprojection.h"
#include "backend/vertex.h"
#include "backend/vertex_pose.h"
#include "../thirdparty/Sophus/sophus/se3.hpp"

using namespace std; 
using namespace myslam;
using namespace Eigen; 

void check_reprojection_intri();
void check_reprojection(); 

int main()
{
	check_reprojection_intri();
	// check_reprojection();
	return 1; 
}

void check_reprojection()
{
	Vector3d pi(0, 0, 0); 
	Quaterniond qi(1, 0, 0, 0); 
	Vector3d pj(0.3, -0.12, 1.1); // 2.1 
	Quaterniond qj(0.917420845, 0.348832, -0.1871413, 0.0391356); 
	Vector3d tic(0, 0, 0); 
	Quaterniond qic(1, 0, 0, 0); 
	
  	// Eigen::Vector3d pts_i(0.5, 0.5, 1.); 
    Eigen::Vector3d pts_i(-1.3163, 0.660394, 1.); 
 	Eigen::Vector3d pts_j(-0.418893, 0.0679902, 1.0);
 
	double lamb_di = 0.470548;

	// pose ext pc2b
	shared_ptr<backend::VertexPose> vertexExt(new backend::VertexPose());
    {
        Eigen::VectorXd pose(7);
        pose << tic[0], tic[1], tic[2], qic.x(), qic.y(), qic.z(), qic.w();
        vertexExt->SetParameters(pose);
    }

	// pose i
	shared_ptr<backend::VertexPose> vertexCam_i(new backend::VertexPose());
	{
		Eigen::VectorXd pose(7);
		pose << pi[0], pi[1], pi[2], qi.x(), qi.y(), qi.z(), qi.w();
		vertexCam_i->SetParameters(pose);
	}
	// pose j 
	shared_ptr<backend::VertexPose> vertexCam_j(new backend::VertexPose());
	{
		Eigen::VectorXd pose(7);
		pose << pj[0], pj[1], pj[2], qj.x(), qj.y(), qj.z(), qj.w();
		vertexCam_j->SetParameters(pose);
	}

	// feature depth 
	shared_ptr<backend::VertexInverseDepth> verterxPoint(new backend::VertexInverseDepth());
    VecX inv_d(1);
    inv_d << lamb_di;
    verterxPoint->SetParameters(inv_d);

	std::shared_ptr<backend::EdgeReprojection> edge(new backend::EdgeReprojection(pts_i, pts_j));
	std::vector<std::shared_ptr<backend::Vertex>> edge_vertex;
	edge_vertex.push_back(verterxPoint);
	edge_vertex.push_back(vertexCam_i);
	edge_vertex.push_back(vertexCam_j);
	edge_vertex.push_back(vertexExt);
	edge->SetVertex(edge_vertex);

	// print out jacobians 
	edge->ComputeResidual(); 
	edge->ComputeJacobians(); 
	vector<MatXX> js = edge->Jacobians();
	VecX resx = edge->Residual();
	Vector2d res(resx[0], resx[1]); 
	for(int i=0; i<js.size(); i++){

		cout <<"dr_dvertex_"<<i+1<<endl
			<<js[i]<<endl; 

	}

	cout << "res: "<<res.transpose()<<endl;
	
	Matrix<double, 2, 1+6+6+6> num_jacobian; 
	double eps = 1e-6;
	// numeric jacobians 
	for(int k=0; k<19; k++){

		Vector3d pi(0, 0, 0); 
		Quaterniond qi(1, 0, 0, 0); 
		Vector3d pj(0.3, -0.12, 1.1); 
		Quaterniond qj(0.917420845, 0.348832, -0.1871413, 0.0391356); 
		Vector3d tic(0, 0, 0); 
		Quaterniond qic(1, 0, 0, 0); 
	
		double inv_dep_i = lamb_di;

		if(k == 0){
			inv_dep_i += eps;
		}else{
			int a = (k-1)/3; int b = (k-1)%3;
			Eigen::Vector3d delta = Eigen::Vector3d(b==0, b==1, b==2) * eps; 
			if(a==0){
				pi += delta;
			}else if(a == 1){
				qi = qi * Sophus::SO3d::exp(delta).unit_quaternion();
				qi.normalized();
			}else if(a == 2){
				pj += delta; 
			}else if(a == 3){
				qj = qj * Sophus::SO3d::exp(delta).unit_quaternion();
				qj.normalized();
			}else if(a == 4){
				tic += delta;
			}else if(a == 5){
				qic = qic * Sophus::SO3d::exp(delta).unit_quaternion();
				qic.normalized();
			}
		}
 	
		Vec3 pts_camera_i = pts_i / inv_dep_i;
    	Vec3 pts_imu_i = qic * pts_camera_i + tic;
    	Vec3 pts_w = qi * pts_imu_i + pi;
    	Vec3 pts_imu_j = qj.inverse() * (pts_w - pj);
    	Vec3 pts_camera_j = qic.inverse() * (pts_imu_j - tic);

    	double dep_j = pts_camera_j.z();
    	Vector2d new_res = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>(); 
    	num_jacobian.col(k) = (new_res - res)/eps;

    	// cout <<" new_res = "<<new_res.transpose()<<" res = "<<res.transpose()<<endl; 
	}

	cout <<" numeric jacobians: "<<endl<< num_jacobian<<endl;

	return ; 
}

void check_reprojection_intri()
{
	Vector3d pi(0, 0, 0); 
	Quaterniond qi(1, 0, 0, 0); 
	Vector3d pj(0.3, -0.12, 1.1); // 2.1 
	Quaterniond qj(0.917420845, 0.348832, -0.1871413, 0.0391356); 
	Vector3d tic(0, 0, 0); 
	Quaterniond qic(1, 0, 0, 0); 

	Vector3d ni(460, 320, 240); // f, cx, cy
	Vector3d nj(470, 310, 230); // f, cx, cy 
  	
  	// Eigen::Vector3d pts_i(0.5, 0.5, 1.); 
    Eigen::Vector3d pts_i(-1.3163, 0.660394, 1.); 
 	Eigen::Vector3d pts_j(-0.418893, 0.0679902, 1.0);
 	Vector3d uvi(pts_i.x()*ni.x() + ni.y(), pts_i.y()*ni.x() + ni.z(), 1.);
 	Vector3d uvj(pts_j.x()*nj.x() + nj.y(), pts_j.y()*nj.x() + nj.z(), 1.);

	double lamb_di = 0.470548;

	// pose ext pc2b
	shared_ptr<backend::VertexPose> vertexExt(new backend::VertexPose());
    {
        Eigen::VectorXd pose(7);
        pose << tic[0], tic[1], tic[2], qic.x(), qic.y(), qic.z(), qic.w();
        vertexExt->SetParameters(pose);
    }

	// pose i
	shared_ptr<backend::VertexPoseIntri> vertexCam_i(new backend::VertexPoseIntri());
	{
		Eigen::VectorXd pose(10);
		pose << pi[0], pi[1], pi[2], qi.x(), qi.y(), qi.z(), qi.w(), ni[0], ni[1], ni[2];
		vertexCam_i->SetParameters(pose);
	}
	// pose j 
	shared_ptr<backend::VertexPoseIntri> vertexCam_j(new backend::VertexPoseIntri());
	{
		Eigen::VectorXd pose(10);
		pose << pj[0], pj[1], pj[2], qj.x(), qj.y(), qj.z(), qj.w(), nj[0], nj[1], nj[2];
		vertexCam_j->SetParameters(pose);
	}

	// feature depth 
	shared_ptr<backend::VertexInverseDepth> verterxPoint(new backend::VertexInverseDepth());
    VecX inv_d(1);
    inv_d << lamb_di;
    verterxPoint->SetParameters(inv_d);

	std::shared_ptr<backend::EdgeReprojectionIntri> edge(new backend::EdgeReprojectionIntri(uvi, uvj));
	std::vector<std::shared_ptr<backend::Vertex>> edge_vertex;
	edge_vertex.push_back(verterxPoint);
	edge_vertex.push_back(vertexCam_i);
	edge_vertex.push_back(vertexCam_j);
	edge_vertex.push_back(vertexExt);
	edge->SetVertex(edge_vertex);

	// print out jacobians 
	edge->ComputeResidual(); 
	edge->ComputeJacobians(); 
	vector<MatXX> js = edge->Jacobians();
	VecX resx = edge->Residual();
	Vector2d res(resx[0], resx[1]); 
	for(int i=0; i<js.size(); i++){

		cout <<"dr_dvertex_"<<i+1<<endl
			<<js[i]<<endl; 

	}

	cout << "res: "<<res.transpose()<<endl;
	
	Matrix<double, 2, 1+9+9+6> num_jacobian; 
	double eps = 1e-6;
	// numeric jacobians 
	for(int k=0; k<25; k++){

		Vector3d pi(0, 0, 0); 
		Quaterniond qi(1, 0, 0, 0); 
		Vector3d pj(0.3, -0.12, 1.1); 
		Quaterniond qj(0.917420845, 0.348832, -0.1871413, 0.0391356); 
		Vector3d tic(0, 0, 0); 
		Quaterniond qic(1, 0, 0, 0); 

		Vector3d ni(460, 320, 240); // f, cx, cy
		Vector3d nj(470, 310, 230); // f, cx, cy 
	
		double inv_dep_i = lamb_di;

		if(k == 0){
			inv_dep_i += eps;
		}else{
			int a = (k-1)/3; int b = (k-1)%3;
			Eigen::Vector3d delta = Eigen::Vector3d(b==0, b==1, b==2) * eps; 
			if(a==0){
				pi += delta;
			}else if(a == 1){
				qi = qi * Sophus::SO3d::exp(delta).unit_quaternion();
				qi.normalized();
			}else if(a == 2){
				ni += delta; 
			}else if(a == 3){
				pj += delta; 
			}else if(a == 4){
				qj = qj * Sophus::SO3d::exp(delta).unit_quaternion();
				qj.normalized();
			}else if(a == 5){
				nj += delta; 
			}else if(a == 6){
				tic += delta;
			}else if(a == 7){
				qic = qic * Sophus::SO3d::exp(delta).unit_quaternion();
				qic.normalized();
			}
		}
  		Eigen::Vector3d ptsi((uvi.x()-ni.y())/ni.x(), (uvi.y()-ni.z())/ni.x(), 1.); 
 		Eigen::Vector3d ptsj((uvj.x()-nj.y())/nj.x(), (uvj.y()-nj.z())/nj.x(), 1.0);
	
		Vec3 pts_camera_i = ptsi / inv_dep_i;
    	Vec3 pts_imu_i = qic * pts_camera_i + tic;
    	Vec3 pts_w = qi * pts_imu_i + pi;
    	Vec3 pts_imu_j = qj.inverse() * (pts_w - pj);
    	Vec3 pts_camera_j = qic.inverse() * (pts_imu_j - tic);

    	double dep_j = pts_camera_j.z();
    	Vector2d new_res = (pts_camera_j / dep_j).head<2>() - ptsj.head<2>(); 
    	num_jacobian.col(k) = (new_res - res)/eps;

    	// cout <<" new_res = "<<new_res.transpose()<<" res = "<<res.transpose()<<endl; 
	}

	cout <<" numeric jacobians: "<<endl<< num_jacobian<<endl;

	return ; 
}