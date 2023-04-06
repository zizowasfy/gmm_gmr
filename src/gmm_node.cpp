/*
 * Copyright (c) 2014, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// custom
#include "gmm_node.h"
#include <gaussian_mixture_model/gmm.h>
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
#include <franka_msgs/FrankaState.h>

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

#include <data_handle/DemonsInfo.h>

class GMMNode
{
  public:
  typedef unsigned int uint;

  GMMNode(ros::NodeHandle & nh): m_nh(nh), m_shutting_down_thread(&GMMNode::shutdownWaitingThread,this)
  {
    int temp_int;
    std::string temp_string;
    double temp_double;

    m_nh.param<int>(PARAM_NAME_GAUSSIAN_COUNT_MAX,temp_int,PARAM_DEFAULT_GAUSSIAN_COUNT_MAX);
    m_gaussian_count_max = (temp_int > 0) ? temp_int : 1;

    m_nh.param<int>(PARAM_NAME_GAUSSIAN_COUNT_MIN,temp_int,PARAM_DEFAULT_GAUSSIAN_COUNT_MIN);
    m_gaussian_count_min = (temp_int > 0) ? temp_int : 1;

    m_nh.param<std::string>(PARAM_NAME_DATA_INPUT_TOPIC,temp_string,PARAM_DEFAULT_DATA_INPUT_TOPIC);
    m_data_subscriber = m_nh.subscribe(temp_string,10,&GMMNode::onData,this);

    m_nh.param<double>(PARAM_NAME_BIC_TERM_THRESHOLD,temp_double,PARAM_DEFAULT_BIC_TERM_THRESHOLD);
    m_bic_termination_threshold = temp_double;

    m_nh.param<int>(PARAM_NAME_TIME_COLUMN_ID,temp_int,PARAM_DEFAULT_TIME_COLUMN_ID);
    m_time_column_id = temp_int >= 0 ? temp_int : PARAM_DEFAULT_TIME_COLUMN_ID;

    m_nh.param<double>(PARAM_NAME_BIC_PARAMS_WEIGHT,temp_double,PARAM_DEFAULT_BIC_PARAMS_WEIGHT);
    m_bic_params_weight = temp_double;

    m_nh.param<std::string>(PARAM_NAME_MIX_OUTPUT_TOPIC,temp_string,PARAM_DEFAULT_MIX_OUTPUT_TOPIC);
    m_mix_publisher = m_nh.advertise<gaussian_mixture_model::GaussianMixture>(temp_string,2);

    client = m_nh.serviceClient<data_handle::DemonsInfo>("/DemonsInfo_Service");

    // learned_posesArray_pub = m_nh.advertise<geometry_msgs::PoseArray>("/gmm/learned_trajectory",1000);

    learned_pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/gmm/learned_pose", 100);

    pose_regression_sub = m_nh.subscribe("/robot_ee_cartesianpose", 100, &GMMNode::poseRegressionCallback, this);

    out_xp.resize(1, 4); out_xp(0,0) = 100000;
  }

  class TerminationHandler: public GMMExpectationMaximization::ITerminationHandler
  {
    public:
    bool isTerminated() {return ros::isShuttingDown(); }
  };

  ~GMMNode()
  {
    ros::shutdown();
    m_shutting_down_thread.join();
  }

  void Run()
  {
    while (!ros::isShuttingDown())
    {
      std_msgs::Float32MultiArrayConstPtr rosdata;

      // scope only
      {
        boost::mutex::scoped_lock lock(m_queue_mutex);
        while (m_queue.empty() && !ros::isShuttingDown())
          m_queue_cond.wait(lock);

        if (ros::isShuttingDown())
          return;

        rosdata = m_queue.front();
        m_queue.pop_front();
      }

      ROS_INFO("gmm: got a message.");

      if (rosdata->layout.dim.size() != 2)
      {
        ROS_INFO_STREAM(rosdata->layout.dim.size());
        ROS_ERROR("gmm: input array must contain two dimensions!!");
        continue;
      }

      const uint dim = rosdata->layout.dim[1].size;
      const uint ndata = rosdata->layout.dim[0].size;
      const uint size = rosdata->data.size();

      if (dim * ndata != size)
      {
        ROS_ERROR("gmm: input array contains %u elements, but that is not the product of individual dimensions %u * %u",
          size,dim,ndata);
        continue;
      }

      // convert input data
      eigendata.resize(ndata,dim);
      
      for (uint i = 0; i < ndata; i++)
        for (uint h = 0; h < dim; h++)
          eigendata(i,h) = rosdata->data[dim * i + h];


      // execute and find the best (lower) bic
      uint best_num_gauss = 0;
      float best_bic = 0.0;
      GMMExpectationMaximization::Ptr best_g;

      if (ros::isShuttingDown())
          return;

      for (uint num_gauss = m_gaussian_count_min; num_gauss <= m_gaussian_count_max; num_gauss++)
      {
        ROS_INFO("gmm: attempting gaussian count: %u",num_gauss);
        if (num_gauss > ndata)
        {
          ROS_WARN("gmm: too few points (%u) to fit %u gaussian(s).",ndata,num_gauss);
          break;
        }

        GMMExpectationMaximization::Ptr gmix(new GMMExpectationMaximization);
        gmix->setTerminationHandler(GMMExpectationMaximization::ITerminationHandler::Ptr(new TerminationHandler));

        gmix->setBicParamsWeight(m_bic_params_weight);
        if (!gmix->execute(num_gauss,m_time_column_id,eigendata))
        {
          ROS_WARN("gmm: EM failed.");
          continue;
        }

        if (ros::isShuttingDown())
          return;

        float bic = gmix->getBIC(eigendata);
        ROS_INFO("gmm: bic is: %f",bic);
        if (best_num_gauss == 0 || bic < best_bic)
        {
          best_bic = bic;
          best_num_gauss = num_gauss;
          best_g = gmix;
        }

        if (ros::isShuttingDown())
          return;

        // the bic is rising: exit
        if (bic - best_bic > m_bic_termination_threshold)
          break;
      }

      if (!best_g)
      {
        ROS_ERROR("gmm: couldn't find any GMM.");
        continue;
      }

      ROS_INFO("gmm: chosen gaussian count %u",best_num_gauss);

      // create the message and send
      means = best_g->getMeans();
      weights = best_g->getWeights();
      covariances = best_g->getCovariances();

      gaussian_mixture_model::GaussianMixturePtr mix_msg = gaussian_mixture_model::GaussianMixturePtr(new gaussian_mixture_model::GaussianMixture);
      mix_msg->bic = best_bic;
      mix_msg->gaussians.resize(best_num_gauss);
      mix_msg->weights.resize(best_num_gauss);
      for (uint s = 0; s < best_num_gauss; s++)
      {
        mix_msg->weights[s] = weights[s];
        gaussian_mixture_model::Gaussian & g = mix_msg->gaussians[s];
        g.means.resize(dim);
        g.covariances.resize(dim * dim);

        for (uint i = 0; i < dim; i++)
          g.means[i] = means[s][i];
        for (uint i = 0; i < dim; i++)
          for (uint h = 0; h < dim; h++)
            g.covariances[i * dim + h] = covariances[s](i,h);
      }

      // doRegression(eigendata,  means, weights , covariances);

      Eigendata.push_back(eigendata);
      Means.push_back(means);
      Weights.push_back(weights);
      Covariances.push_back(covariances);

      m_mix_publisher.publish(mix_msg);
      ROS_INFO("gmm: message sent.");
      data_received++;
      srv.request.read = false;
      if (data_received == 1 && client.call(srv)) 
      {
        maxRange = srv.response.maxRange;
        minRange = srv.response.minRange;
      }
    }
  }

  void onData(std_msgs::Float32MultiArrayConstPtr data)
  {
    boost::mutex::scoped_lock lock(m_queue_mutex);
    m_queue.push_back(data);
    m_queue_cond.notify_one();
  }

  void shutdownWaitingThread()
  {
    ros::waitForShutdown();

    // scope only
    {
      boost::mutex::scoped_lock lock(m_queue_mutex);
      m_queue_cond.notify_all();
    }
  }
  
  // this function compute PDF of X which has lenght of k
  float computeNormalDistributionPDF(Eigen::VectorXf X, Eigen::VectorXf mean, Eigen::MatrixXf covariance)
  {
      float nominator, denominator, tmp;


      Eigen::MatrixXf tmp_nominator_matrixXf;
      tmp_nominator_matrixXf=( X.transpose()-mean.transpose() )  * covariance.inverse() *  (X-mean) ;
      tmp=-0.5*tmp_nominator_matrixXf(0,0);
      nominator=std::exp (tmp);

      int k=X.rows();

      tmp=std::pow(2*M_PI,k) *covariance.determinant();
      denominator=std::pow( tmp, 0.5 );
      return  (nominator/denominator);
  }
  
  void doRegression(Eigen::MatrixXf  xi, const std::vector<Eigen::VectorXf>  means, const std::vector<float>  weights , const std::vector<Eigen::MatrixXf>  covariances)
  {
    std::cout << "*** Starting Regression ***    " << std::endl;
    
    int numTrajsamples = 1;     // number of points in the chosen trajectory
    int i = 0;
    std::string trajDir;
    geometry_msgs::PoseArray learned_posesArray;
    geometry_msgs::PoseStamped pose;
    rosbag::Bag rbag, wbag;

    Eigen::VectorXf X;
    Eigen::VectorXf mean;
    Eigen::MatrixXf covariance;

    Eigen::MatrixXf out_xi;

    // out_xi.resize(xi.rows() ,xi.cols());
    // out_xi=Eigen::MatrixXf::Zero(xi.rows(), xi.cols());
    // numTrajsamples = 703;
    out_xi.resize(numTrajsamples, xi.cols());
    out_xi=Eigen::MatrixXf::Zero(numTrajsamples, xi.cols());

    // std::cout << "numTrajsamples:" << numTrajsamples << std::endl;
    // std::cout<< "xi:" <<std::endl << xi.size()/4 <<std::endl;
    // std::cout<< "means.size(): " << means.size() <<std::endl;
    // std::cout<< "covariances: " << covariances.size() <<std::endl;
    // std::cout<< "weights: " << weights.size() <<std::endl;



  //equation 10
    float xi_hat_s_k, mu_s_k,mu_t_k,sigma_st_k,inv_sigma_t_k,sigma_hat_s_k,
          sigma_s_k, sigma_t_k,sigma_ts_k, xi_t, beta_k,xi_hat_s,beta_k_sum, beta_k_xi_hat_s_k_sum;

    for(std::size_t coord=1; coord<out_xi.cols(); coord++)    // loop over the xyz coordinates
    {
      beta_k_sum=0;
      beta_k_xi_hat_s_k_sum=0;
      for(std::size_t k=0; k<means.size(); k++)     // loop over the number of gaussians
      {
        mu_t_k=means.at(k)(0);
        mu_s_k=means.at(k)(coord);
        sigma_st_k=covariances.at(k)(coord,0);
        sigma_t_k=covariances.at(k)(0,0);
        inv_sigma_t_k=1/sigma_t_k;
        sigma_s_k=covariances.at(k)(coord,coord);
        sigma_ts_k=covariances.at(k)(0,coord);
        // xi_t=xi(i,0);
        // xi_t = i;
        // xi_t = sqrt(pow(mp->O_T_EE_c[12],2) + pow(mp->O_T_EE_c[13],2) + pow(mp->O_T_EE_c[14],2));
        xi_t = pose_regress;
        // xi_t = xi(i,3);
        xi_hat_s_k=mu_s_k+ sigma_st_k*inv_sigma_t_k*(xi_t -mu_t_k);
        sigma_hat_s_k=sigma_s_k -sigma_st_k*inv_sigma_t_k*sigma_ts_k;

        //equation 11

        X.resize(1,1);
        mean.resize(1,1);
        covariance.resize(1,1);

        X<<xi_t;
        mean<<mu_t_k;

        covariance<<sigma_t_k;

        beta_k=weights.at(k) * computeNormalDistributionPDF(X, mean,covariance);
        beta_k_sum=beta_k_sum+beta_k;

        xi_hat_s=beta_k*xi_hat_s_k;
        beta_k_xi_hat_s_k_sum=beta_k_xi_hat_s_k_sum+xi_hat_s;
      }
      out_xi(0,0)=xi_t;
      out_xi(0,coord)=beta_k_xi_hat_s_k_sum/beta_k_sum;
    }
    
    // // Trying to move in the opposite direction if outside the GMM
    // // if (out_xi(0,0) - out_xp(0,0) > 0.001)
    // if (out_xi(0,0) > 0.456)
    // {
    //   Eigen::MatrixXf out_xd = out_xi - out_xp;
    //   std::cout << "out_xd = " << std::endl << out_xd << std::endl;

    //   pose.pose.position.x = out_xi(0,1)-out_xd(0,1); pose.pose.position.y = out_xi(0,2)-out_xd(0,2); pose.pose.position.z = out_xi(0,3)-out_xd(0,3);
    //   std::cout << "out_xp(0,0) = " << out_xp(0,0) << std::endl;
    // }
    // //\ Trying to move in the opposite direction if outside the GMM  
    
    poseArr.push_back(out_xi(0,1));

    // pose.pose.position.x = out_xi(0,1); pose.pose.position.y = out_xi(0,2); pose.pose.position.z = out_xi(0,3);

    // // Publish the learned_trajectory on topic /gmm_node/gmm/learned_trajectory
    // pose.header.frame_id = "panda_link0";
    // learned_pose_pub.publish(pose);

    // out_xp = out_xi;
    // debugfile.open("debug_regression.txt");
    // debugfile << out_xi(0,0);// << " " << out_xi(0,1) << " " << out_xi(0,2) << " " << out_xi(0,2) << "\n";
    // // debugfile.close();
    ros::Duration(0.1).sleep();

    std::cout<< "out_xi:" << std::endl << out_xi <<std::endl;
    std::cout<< "out_xi.rows():"  <<out_xi.rows() <<std::endl;
    std::cout<< "out_xi.cols():"  <<out_xi.cols() <<std::endl;
    std::cout<< "-------------------------------------" <<std::endl;
  }

  void poseRegressionCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    // pose_regress = sqrt(pow(msg->pose.position.x,2) + pow(msg->pose.position.y,2) + pow(msg->pose.position.z,2));
    double msgArr[3] = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    double errmsgArr[3] = {0.5502645502645548 - msg->pose.position.x, -0.009326424870472083 - msg->pose.position.y, 0.08959276018099538 - msg->pose.position.z};
    // double errmsgArr[3] = {0.7 - msg->pose.position.x, 0.4 - msg->pose.position.y, 0.4 - msg->pose.position.z};
    double errmsgArrDist = sqrt(pow(errmsgArr[0],2) + pow(errmsgArr[1],2) + pow(errmsgArr[2],2));
    char axis[3] = {'x', 'y', 'z'};
    // if (errmsgArrDist > 0.05 && data_received == 3)
    if (data_received == 3)
    {
      for (int i = 0; i < 3; i++)
      {
        if (abs(errmsgArr[i]) < 0.01) 
        {
          poseArr[i] = 0.0;
          ROS_INFO(" $$$$$$$$$$$$$$$ Axis '%c' reached the GOAL Position", axis[i]);
          continue;
        }
        if (msgArr[i] > maxRange[i])
        {
          // std::cout << maxRange[i] << std::endl;
          ROS_ERROR(" Axis '%c' is OUT of the MAX Range. %f | %f ", axis[i],msgArr[i],maxRange[i]);
        }
        if (msgArr[i] < minRange[i])
        {
          ROS_ERROR(" Axis '%c' is OUT of the MIN Range. %f | %f ", axis[i],msgArr[i],minRange[i]);
        }
        pose_regress = errmsgArr[i];
        std::cout << "*** DOING Regression ***" << std::endl;
        std::cout << " Regression Axis --> " << axis[i] << std::endl;
        doRegression(Eigendata[i],  Means[i], Weights[i], Covariances[i]);
        // data_received = false;
      }
      // std::cout << "poseArray = " << poseArr[0] << " " << poseArr[1] << " " << poseArr[2] << std::endl;
      learned_deltaPose.header.frame_id = "panda_link0";
      learned_deltaPose.pose.position.x = poseArr[0]*50; learned_deltaPose.pose.position.y = poseArr[1]*50; learned_deltaPose.pose.position.z = poseArr[2]*50;
      learned_pose_pub.publish(learned_deltaPose);
      poseArr.clear();
    }

  }

  private:
  ros::NodeHandle & m_nh;

  boost::mutex m_queue_mutex;
  boost::condition_variable m_queue_cond;
  std::deque<std_msgs::Float32MultiArrayConstPtr> m_queue;

  uint m_gaussian_count_max;
  uint m_gaussian_count_min;

  uint m_time_column_id;

  float m_bic_termination_threshold;
  float m_bic_params_weight;

  ros::Subscriber m_data_subscriber;
  ros::Publisher m_mix_publisher;

  int data_received = 0;
  Eigen::MatrixXf eigendata;
  std::vector<Eigen::VectorXf> means;
  std::vector<float> weights;
  std::vector<Eigen::MatrixXf> covariances;
  // --- 
  std::vector<Eigen::MatrixXf> Eigendata;
  std::vector<std::vector<Eigen::VectorXf>> Means;
  std::vector<std::vector<float>> Weights;
  std::vector<std::vector<Eigen::MatrixXf>> Covariances;
  // --
  std::vector<double> poseArr = {0,0,0};
  geometry_msgs::PoseStamped learned_deltaPose;
  // --
  ros::ServiceClient client;
  data_handle::DemonsInfo srv;
  ros::Publisher learned_posesArray_pub;
  ros::Publisher learned_pose_pub;
  ros::Subscriber pose_regression_sub;    // Receives the current pose of the robot to regress over each pose
  double pose_regress;
  Eigen::MatrixXf out_xp;
  std::vector<float> minRange, maxRange;

  fstream debugfile;

  // this thread will simply wait for shutdown
  // and unlock all the conditions variables
  boost::thread m_shutting_down_thread;
};

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"gmm_node");

  ros::NodeHandle nh("~");
  GMMNode node(nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  node.Run();

  return 0;
}
