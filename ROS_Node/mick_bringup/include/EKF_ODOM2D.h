#ifndef EKF_ODOM2D_H_
#define EKF_ODOM2D_H_

#include <iostream>
#include <vector>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Geometry> 
 
using namespace std;
 
namespace EKF
{
class EKF_ODOM2D
{
public:
// 设置初始航向角和2D位置
    EKF_ODOM2D(const float& x,const float& y, const float& theta);

    bool Run(const double& dt,const float& o_v,const float& o_w,const float& yaw);

    Eigen::Vector3d get2DPose(void);    
    void Param_Change(float noise_R, float noise_Q);

    void Release();
    void RequestStop();
    void RequestStart();
    bool Stop();
    bool isStopped();

private:
 
    double x_,y_,theta_;
    Eigen::Matrix<double, 3, 1> state_X;   // x,y,theta
    Eigen::Matrix<double, 3, 3> P,Q ;
    double  R ;

    bool mbFlagInit;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;
};
}
 

#endif