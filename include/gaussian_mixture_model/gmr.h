#ifndef GMR_H
#define GMR_H

// custom
// #include "gmm_node.h"
// #include <gaussian_mixture_model/gmm.h>
#include <gaussian_mixture_model/GaussianMixture.h>
#include <gaussian_mixture_model/Gaussian.h>

// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h> 

// Boost
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// STL
#include <string>
#include <deque>

// Eigen
#include <Eigen/Dense>

//
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
using namespace boost::filesystem;

// class GMR
// {
//   public:

//     typedef unsigned int uint;

//     void onTPGMM(gaussian_mixture_model::GaussianMixturePtr data);

//     static float computeNormalDistributionPDF(Eigen::VectorXf X, Eigen::VectorXf mean, Eigen::MatrixXf covariance);

//     static void doRegression(Eigen::MatrixXf& xi,
//                         const std::vector<Eigen::VectorXf>&  means,
//                         const std::vector<float>&  weights,
//                         const std::vector<Eigen::MatrixXf>& covariances,
//                         geometry_msgs::PoseArray& learned_posesArray);

// };

namespace GMR
{
    typedef unsigned int uint;

    // This function shall only be used a template. It has to be created and run from the node from which you will receive TPGMM
    void onTPGMM(gaussian_mixture_model::GaussianMixturePtr data);

    float computeNormalDistributionPDF(Eigen::VectorXf X, Eigen::VectorXf mean, Eigen::MatrixXf covariance);

    void doRegression(Eigen::MatrixXf& xi,
                        const std::vector<Eigen::VectorXf>&  means,
                        const std::vector<float>&  weights,
                        const std::vector<Eigen::MatrixXf>& covariances,
                        geometry_msgs::PoseArray& learned_posesArray);

};

#endif // GMR_H