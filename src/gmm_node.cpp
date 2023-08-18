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

    demonsInfo_client = m_nh.serviceClient<data_handle::DemonsInfo>("/DemonsInfo_Service");

    learned_posesArray_pub = m_nh.advertise<geometry_msgs::PoseArray>("/gmm/learned_trajectory",10);

    learned_posesArray_sub = m_nh.subscribe("/gmm/learned_trajectory", 10, &GMMNode::saveLearnedTraj, this);

    learned_pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/gmm/learned_pose", 1);

    // pose_regression_sub = m_nh.subscribe("/robot_ee_cartesianpose", 1, &GMMNode::poseRegressionCallback, this);

    tpgmm_sub = m_nh.subscribe("/gmm/mix", 1, &GMMNode::onTPGMM, this);

    m_nh.getParam("/task_param", task_param);
    m_nh.getParam("/subtask_param", subtask_param);
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
    std::cout << "GMMNode is DYYYIIIIING" << std::endl;
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

      m_mix_publisher.publish(mix_msg);
      ROS_INFO("gmm: message sent.");
      // data_recevied = true;
      trajRegressionCallback();
    }
  }

  // Receiving TPGMM from tp_gmm node and put it in the format for doRegression()
  void onTPGMM(gaussian_mixture_model::GaussianMixturePtr data)
  {
    int gmm_num = data->weights.size();
    int g_dim;
    std::vector<float> tpgmm_weights = data->weights;
    std::vector<Eigen::VectorXf> tpgmm_means;
    std::vector<Eigen::MatrixXf> tpgmm_covariances;

    for (int n = 0; n < gmm_num; n++)
    {
      g_dim = data->gaussians[n].means.size();
      Eigen::VectorXf g_means(g_dim);
      Eigen::MatrixXf g_covariances(g_dim, g_dim);

      std::cout << "g_means.size() = " << g_means.size() << std::endl;
      int i = 0;
      for (int m = 0; m < g_dim; m++)
      {
        g_means(m) = data->gaussians[n].means[m];
        for(int c = 0; c < g_dim; c++)
        {
          g_covariances(m,c) = data->gaussians[n].covariances[i];
          i++;
        }
      }
      tpgmm_means.push_back(g_means);
      tpgmm_covariances.push_back(g_covariances);
    }
    
    Eigen::MatrixXf Xi(int(data->bic), g_dim);
    doRegression(Xi, tpgmm_means, tpgmm_weights, tpgmm_covariances);
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
    std::cout << "    *** Starting Regression ***    " << std::endl;
    
    // int numTrajsamples = 1;     // number of points in the chosen trajectory
    // int i = 0;
    // std::string trajDir;

    // rosbag::Bag rbag, wbag;

    geometry_msgs::PoseArray learned_posesArray;
    geometry_msgs::Pose pose;

    Eigen::VectorXf X;
    Eigen::VectorXf mean;
    Eigen::MatrixXf covariance;

    Eigen::MatrixXf out_xi;

    // out_xi.resize(xi.rows() ,xi.cols());
    // out_xi=Eigen::MatrixXf::Zero(xi.rows(), xi.cols());
    // numTrajsamples = 703;
    out_xi.resize(xi.rows(), xi.cols());
    out_xi=Eigen::MatrixXf::Zero(xi.rows(), xi.cols());

    // std::cout << "numTrajsamples:" << numTrajsamples << std::endl;
    // std::cout<< "xi:" <<std::endl << xi.size()/4 <<std::endl;
    // std::cout<< "means.size(): " << means.size() <<std::endl;
    // std::cout<< "covariances: " << covariances.size() <<std::endl;
    // std::cout<< "weights: " << weights.size() <<std::endl;

  //equation 10
    float xi_hat_s_k, mu_s_k,mu_t_k,sigma_st_k,inv_sigma_t_k,sigma_hat_s_k,
          sigma_s_k, sigma_t_k,sigma_ts_k, xi_t, beta_k,xi_hat_s,beta_k_sum, beta_k_xi_hat_s_k_sum;

    for(std::size_t i = 0; i < out_xi.rows(); i++)   // loop over data points
    {
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
          xi_t = i;
          // xi_t = sqrt(pow(mp->O_T_EE_c[12],2) + pow(mp->O_T_EE_c[13],2) + pow(mp->O_T_EE_c[14],2));
          // xi_t = pose_regress;
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
        out_xi(i,0)=xi_t;
        out_xi(i,coord)=beta_k_xi_hat_s_k_sum/beta_k_sum;
      }

      pose.position.x = out_xi(i,1);
      pose.position.y = out_xi(i,2);
      pose.position.z = out_xi(i,3);
      learned_posesArray.poses.push_back(pose);
    }

    learned_posesArray.header.frame_id = "panda_link0";
    learned_posesArray_pub.publish(learned_posesArray);

    // // Publish the learned_trajectory on topic /gmm_node/gmm/learned_trajectory
    // learned_pose.header.frame_id = "panda_link0";
    // learned_pose_pub.publish(learned_pose);
    // ros::Duration(0.01).sleep();

    std::cout<< "-------------------------------------" <<std::endl;

    std::cout<< "out_xi:" <<std::endl <<out_xi <<std::endl;

    std::cout<< "out_xi.rows():"  <<out_xi.rows() <<std::endl;
    std::cout<< "out_xi.cols():"  <<out_xi.cols() <<std::endl;
  }

  void poseRegressionCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    if (data_recevied && !goal_reached)
    {
      // double msgArr[3] = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
      if (subtask_param == "action")
      {
        pose_regress = sqrt(pow(0.5502645502645548 - msg->pose.position.x,2) + pow(-0.009326424870472083 - msg->pose.position.y,2) + pow(0.08959276018099538 - msg->pose.position.z,2));
      }
      else if (subtask_param == "place")
      {
        pose_regress = sqrt(pow(0.7 - msg->pose.position.x,2) + pow(0.4 - msg->pose.position.y,2) + pow(0.4 - msg->pose.position.z,2));
      }
      
      std::cout << "pose_regress: " << pose_regress << std::endl;
      // std::cout << "m_queue.size(): " << m_queue.size() << std::endl;
      // std::cout << "covariances.size(): " << covariances.size() << std::endl;

      if (pose_regress > 0.03)
      {
        std::cout << "*** DOING Regression ***" << std::endl;
        doRegression(eigendata,  means, weights , covariances);
        //Publish the learned_trajectory on topic /gmm/learned_trajectory to move robot
        learned_pose.header.frame_id = "delta";
        learned_pose_pub.publish(learned_pose);
        learned_posesArray.poses.push_back(addPoses(learned_pose.pose, msg->pose));
        ros::Rate(100).sleep();   // Still not sure what freq we collected the demons in!!
      }
      else
      {
        goal_reached = true;
        learned_pose.pose.position.x = 0.0;
        learned_pose.pose.position.y = 0.0;
        learned_pose.pose.position.z = 0.0;
        learned_pose_pub.publish(learned_pose);
        learned_posesArray.header.frame_id = "panda_link0";
        learned_posesArray_pub.publish(learned_posesArray);
        ROS_INFO("$$$$$$$$$$$$$$$ Reached the GOAL Position");
      }      
    }
  }

  void trajRegressionCallback()
  {
    geometry_msgs::Pose regress_pose; 
    data_handle::DemonsInfo srv;
    srv.request.send = false;
    if (demonsInfo_client.call(srv))
    {
      rosbag::Bag rbag;
      // Regress over the poseError samples of the longest Demon
      rbag.open(demonsDir + task_param + "/" + subtask_param + "/Franka_Stage2_" + task_param + "_" + subtask_param + "_" + srv.response.names[srv.response.shortest_idx] + ".bag");
      int i = 0;
      for(rosbag::MessageInstance const m: rosbag::View(rbag))
      {
        franka_msgs::FrankaState::ConstPtr mp = m.instantiate<franka_msgs::FrankaState>();   

        pose_regress = mp->O_T_EE_c[0];   // Train over time samples instead of poseError    
        // pose_regress = sqrt(pow(mp->O_T_EE_c[12],2) + pow(mp->O_T_EE_c[13],2) + pow(mp->O_T_EE_c[14],2)); // euclidean distance of the delta position;
        std::cout << "pose_regress = " << pose_regress << std::endl;
        if (mp != nullptr) // && pose_regress > 0.03)
        {
          // Set the starting pose to be the first pose of the longest Demon
          if (i == 0)
          {
            regress_pose.position.x = mp->O_T_EE[12];
            regress_pose.position.y = mp->O_T_EE[13];
            regress_pose.position.z = mp->O_T_EE[14];
            regress_pose.orientation.x = mp->O_T_EE_c[3];
            regress_pose.orientation.y = mp->O_T_EE_c[7];
            regress_pose.orientation.z = mp->O_T_EE_c[11];
            regress_pose.orientation.w = mp->O_T_EE_c[15];                                    
          }

          std::cout<< "means.size(): " << means.size() <<std::endl;        
          std::cout << "*** DOING Regression ***" << std::endl;
          doRegression(eigendata,  means, weights , covariances);

          //Publish the learned_trajectory on topic /gmm/learned_trajectory to move robot
          regress_pose = addPoses(learned_pose.pose, regress_pose);
          learned_posesArray.poses.push_back(regress_pose);
          // ros::Duration(0.1).sleep();
          i++;
        }        
      }
      ROS_INFO_STREAM("Number of Regression Samples: " << i);
      learned_posesArray.header.frame_id = "panda_link0";
      learned_posesArray_pub.publish(learned_posesArray);
      ROS_INFO("$$$$$$$$$$$$$$$ GMM-GMR Learned Trajectory is Published!");
      rbag.close();
    }
  }

  // void trajRegressionCallback()
  // {
  //   geometry_msgs::Pose regressed_pose, end_pose; 
  //   data_handle::DemonsInfo srv;
  //   srv.request.send = false;
  //   if (demonsInfo_client.call(srv))
  //   {
  //     rosbag::Bag rbag;
  //     // Regress over the poseError samples of the longest Demon
  //     rbag.open(demonsDir + task_param + "/" + subtask_param + "/Franka_Stage2_" + task_param + "_" + subtask_param + "_" + srv.response.names[srv.response.longest_idx] + ".bag");
      
  //     int i = 0;
  //     for(rosbag::MessageInstance const m: rosbag::View(rbag))
  //     {
  //       franka_msgs::FrankaState::ConstPtr mp = m.instantiate<franka_msgs::FrankaState>();   
  //       // Set the starting pose for regression to be the first pose of the longest Demon
  //       if (i == 0)
  //       {
  //         regressed_pose.position.x = mp->O_T_EE[12];
  //         regressed_pose.position.y = mp->O_T_EE[13];
  //         regressed_pose.position.z = mp->O_T_EE[14];
  //       }
  //       // Set the ending pose for regression to be the last pose of the longest Demon
  //       else if (i == srv.response.numofposes[srv.response.longest_idx]-1) // -1
  //       {
  //         end_pose.position.x = mp->O_T_EE[12];
  //         end_pose.position.y = mp->O_T_EE[13];
  //         end_pose.position.z = mp->O_T_EE[14];
  //       }
  //       i++;
  //     }
  //     std::cout << "regressed_pose.position.z  = " << regressed_pose.position.z  << std::endl;
  //     std::cout << "end_pose.position.z  = " << end_pose.position.z  << std::endl;

  //     pose_regress = 1000; // initialize with large number
  //     while (pose_regress > 0.08)
  //     {
  //       geometry_msgs::Pose error_pose = subtPoses(end_pose, regressed_pose);
  //       pose_regress = sqrt(pow(error_pose.position.x,2) + pow(error_pose.position.y,2) + pow(error_pose.position.z,2)); // euclidean distance of the delta position;
  //       std::cout << "pose_regress = " << pose_regress << std::endl;
  //       {          
  //         std::cout << "*** DOING Regression ***" << std::endl;
  //         doRegression(eigendata,  means, weights , covariances);

  //         //Publish the learned_trajectory on topic /gmm/learned_trajectory to move robot
  //         regressed_pose = addPoses(learned_pose.pose, regressed_pose);
  //         learned_posesArray.poses.push_back(regressed_pose);
  //         // ros::Duration(0.1).sleep();
  //       }        
  //     }
  //     ROS_INFO_STREAM("Number of Regression Samples: " << i);
  //     learned_posesArray.header.frame_id = "panda_link0";
  //     learned_posesArray_pub.publish(learned_posesArray);
  //     ROS_INFO("$$$$$$$$$$$$$$$ GMM-GMR Learned Trajectory is Published!");
  //     rbag.close();
  //   }
  // }


  void saveLearnedTraj(const geometry_msgs::PoseArrayConstPtr& msg)
  {
    char save;
    std::cout << "\nDo you want to save this gmr_learned_trajectory? (y/n) "; std::cin >> save; std::cout << " " << std::endl;
    if (save == 'y')
    {
      std::cout << "Saving ... !" << std::endl;
      rosbag::Bag wbag;
      wbag.open(llDir + task_param + "/gmm-gmr/gmr_learned_" + subtask_param + "test.bag", rosbag::bagmode::Write);
      
      wbag.write("/gmm/learned_trajectory", ros::Time::now(), msg);
      wbag.close();
      std::cout << "... Saved !" << std::endl;
    }
    else 
    {std::cout << "... Not Saved !" << std::endl;}
  }

  geometry_msgs::Pose addPoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
  {
    geometry_msgs::Pose p;
    Eigen::Quaterniond p_quat, p1_quat, p2_quat;

    p1_quat.coeffs() << p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w;
    p2_quat.coeffs() << p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w;

    p.position.x = p1.position.x + p2.position.x;
    p.position.y = p1.position.y + p2.position.y;
    p.position.z = p1.position.z + p2.position.z;
    p_quat = p1_quat * p2_quat; // desired = delta * current
    p.orientation.x = p_quat.x();
    p.orientation.y = p_quat.y();
    p.orientation.z = p_quat.z();
    p.orientation.w = p_quat.w();

    return p;
  }
  geometry_msgs::Pose subtPoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
  {
    geometry_msgs::Pose p;
    p.position.x = p1.position.x - p2.position.x;
    p.position.y = p1.position.y - p2.position.y;
    p.position.z = p1.position.z - p2.position.z;
    return p;
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

  bool data_recevied = false;
  bool goal_reached = false;
  Eigen::MatrixXf eigendata;
  std::vector<Eigen::VectorXf> means;
  std::vector<float> weights;
  std::vector<Eigen::MatrixXf> covariances;

  ros::ServiceClient demonsInfo_client;
  data_handle::DemonsInfo srv;
  ros::Publisher learned_posesArray_pub;
  ros::Subscriber learned_posesArray_sub;
  ros::Publisher learned_pose_pub;
  ros::Subscriber pose_regression_sub;    // Receives the current pose of the robot to regress over each pose
  ros::Subscriber tpgmm_sub;
  float pose_regress;
  geometry_msgs::PoseStamped learned_pose;
  geometry_msgs::PoseArray learned_posesArray;
  std::string task_param, subtask_param;

  std::string demonsDir = "/home/zizo/Disassembly Teleop/Demons/";
  std::string llDir = "/home/zizo/Disassembly Teleop/LL/";
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
