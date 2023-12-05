#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "EKF_ODOM2D.h"
 

using namespace std;
using namespace Eigen;

namespace EKF
{

EKF_ODOM2D::EKF_ODOM2D(const float& x,const float& y, const float& theta):
        mbStopped(false), mbStopRequested(false)
{
	x_ = x;
	y_ = y;
	theta_ = theta;
	cout << "EKF_ODOM2D initialization!" << endl;

	P = MatrixXd::Identity(3,3);
	R = 1e-1 ;
	Q = 1e-2*MatrixXd::Identity(3,3);

	state_X = MatrixXd::Zero(3,1);
	state_X(0) = x_;
	state_X(1) = y_;
	state_X(2) = theta_;
}
 
bool EKF_ODOM2D::Run(const double& dt,const float& o_v,const float& o_w,const float& yaw)
{
	state_X(0)=  state_X(0) + cos( state_X(2))*o_v*dt;
	state_X(1)=  state_X(1) + sin( state_X(2))*o_v*dt;
	state_X(2)=  state_X(2) + o_w*dt;

	//2. F     Compute discrete transition and noise covariance matrix
	Matrix<double, 3,3> F = Matrix<double, 3,3>::Zero();
	F(0,0) = 1;
	F(0,2) = -sin(state_X(3))*o_v*dt; 
	F(1,1) = 1;
	F(1,2) = cos(state_X(3))*o_v*dt; 
	F(2,2) = 1;
 
	P = F*P*F.transpose()+Q;

	Eigen::Matrix<double, 1, 3> H ;
	H <<  0, 0, 1;
	double   r_thin  =  yaw - H* Eigen::Vector3d(0, 0, 1);

	double S = H*P*H.transpose() + R;
	Eigen::MatrixXd K = P*H.transpose()/S;
	// Compute the error of the state.
	state_X = state_X+ K * r_thin;

	P = (Matrix<double, 3, 3>::Identity()-K*H)*P;
	P= (P +P.transpose()) / 2.0;
	return true;
}
 
Eigen::Vector3d EKF_ODOM2D::get2DPose(void)
{
	return Eigen::Vector3d(x_,y_,theta_);
}
void EKF_ODOM2D::Param_Change(float noise_R, float noise_Q)
{
	if (isStopped())
	{
		R = noise_R;
		Q = noise_Q*MatrixXd::Identity(3,3);
	}
}
 
void EKF_ODOM2D::RequestStop()
{
	unique_lock<mutex> lock(mMutexStop);
	mbStopRequested = true;
}

void EKF_ODOM2D::RequestStart()
{
	unique_lock<mutex> lock(mMutexStop);
	if (mbStopped)
	{
		mbStopped = false;
		mbStopRequested = false;
	}

}

bool EKF_ODOM2D::Stop()
{
	unique_lock<mutex> lock(mMutexStop);
	if (mbStopRequested)
	{
		mbStopped = true;
		return true;
	}
	return false;
}

bool EKF_ODOM2D::isStopped()
{
	unique_lock<mutex> lock(mMutexStop);
	return mbStopped;
}

void EKF_ODOM2D::Release()
{
	unique_lock<mutex> lock(mMutexStop);
	mbStopped = false;
	mbStopRequested = false;
 
	cout << "EKF Odom 2D release " << endl;
}

}