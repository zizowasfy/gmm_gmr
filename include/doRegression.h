
#ifndef DO_REGRESSION_
#define DO_REGRESSION_

// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h> 

// STL
#include <string>
#include <deque>

// Eigen
#include <Eigen/Dense>

// gmm
#include <gaussian_mixture_model/Gaussian.h>
#include <gaussian_mixture_model/GaussianMixture.h>

// system
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
using namespace boost::filesystem;

#define LL_DIR "/home/zizo/Disassembly Teleop/LL/"
std::string l_gmm_dir;
std::string* l_gmm_dir_ptr = &l_gmm_dir;

void getDirfromTxt()
{
  std::string line;
  ifstream File(std::string(LL_DIR) + "learned_gmm_dir.txt");
  while (getline (File, line))
  {
    std::cout << line << std::endl;
  }
  *l_gmm_dir_ptr = line;
}

void getGMMfromBag(Eigen::MatrixXf &GMM_X, std::vector<Eigen::VectorXf> &GMM_means, std::vector<float> &GMM_weights, std::vector<Eigen::MatrixXf> &GMM_covariances)
{
  getDirfromTxt();
  
  // Getting the GMM from the .bag file
  rosbag::Bag rbag;
  // rbag.open("/home/zizo/Disassembly Teleop/LL/Rcover/gmm-gmr/gmm_mix_action2.bag");
  // rbag.open("/home/zizo/Disassembly Teleop/LL/Rbolts/gmm-gmr/learned_gmm_action_test.bag");
  rbag.open(l_gmm_dir + "_learned_gmm_mix" + ".bag");

  // // Initializing Variables
  // Eigen::MatrixXf GMM_X;
  // std::vector<Eigen::VectorXf> GMM_means;
  // std::vector<Eigen::MatrixXf> GMM_covariances;
  // std::vector<float> GMM_weights;

  for(rosbag::MessageInstance const m: rosbag::View(rbag))
  {
    gaussian_mixture_model::GaussianMixture::ConstPtr i = m.instantiate<gaussian_mixture_model::GaussianMixture>();
    if (i != nullptr)
    {
      int GMM_dim = sizeof(i->gaussians[0].means)/3; // 3 cause it seems the size of float32 is 3 bytes instead of 4 bytes!!
      ROS_INFO("GMM Dimension: %d", GMM_dim);

      // Read GaussianMixutre msg and convert it to a format that doRegression(....) accepts
      GMM_X = Eigen::MatrixXf::Identity(1,GMM_dim); // Not really necessary, it's just only to abide by doRegression(....) format
      GMM_means.clear();
      GMM_covariances.clear();
      Eigen::VectorXf G_means(GMM_dim);
      Eigen::MatrixXf G_covariances(GMM_dim, GMM_dim);      
      GMM_weights = i->weights;
      for (gaussian_mixture_model::Gaussian gaussian : i->gaussians)
      {
        // Eigen::VectorXf G_means(GMM_dim);
        // Eigen::MatrixXf G_covariances(GMM_dim, GMM_dim);
        int i = 0;
        for (int d = 0; d < GMM_dim; d++)
        {
          G_means(d) = gaussian.means[d];

          for (int c = 0; c < GMM_dim; c++)
          {
            G_covariances(d,c) = gaussian.covariances[i]; //[c+i]
          }
          i++; //=+ GMM_dim; // To make sure that each row from gaussian.coavariances is placed correctly in G_covariances
        }
        GMM_means.push_back(G_means);
        GMM_covariances.push_back(G_covariances);
      }
    }
  }
  rbag.close();

  rosbag::Bag wbag;
  // wbag.open("/home/zizo/Disassembly Teleop/LL/Rcover/gmm-gmr/GMMsamples.bag", rosbag::bagmode::Write);
  // wbag.open(l_gmm_dir + "_learned_gmm_samples" + ".bag", rosbag::bagmode::Write);
  wbag.open(l_gmm_dir + "_learned_gmm_joint_samples" + ".bag", rosbag::bagmode::Write);
  wbag.close();
}

// // Save samples to rosbag for later visualize
// void saveSamples(std::vector<double> & sample_pnt)
// {
//   rosbag::Bag wbag;
//   // wbag.open("/home/zizo/Disassembly Teleop/LL/Rcover/gmm-gmr/GMMsamples.bag", rosbag::bagmode::Append);
//   wbag.open(l_gmm_dir + "_learned_gmm_samples" + ".bag", rosbag::bagmode::Append);

//   geometry_msgs::PoseStamped sample;
//   sample.header.frame_id = "panda_link0";
//   sample.pose.position.x = sample_pnt[0];
//   sample.pose.position.y = sample_pnt[1];
//   sample.pose.position.z = sample_pnt[2];

//   wbag.write(("/samples"), ros::Time::now(), sample);
//   wbag.close();
// }

// Save samples to rosbag for later visualize
void saveSamples(std::vector<double> & sample_pnt)
{
  rosbag::Bag wbag;
  // wbag.open("/home/zizo/Disassembly Teleop/LL/Rcover/gmm-gmr/GMMsamples.bag", rosbag::bagmode::Append);
  // wbag.open(l_gmm_dir + "_learned_gmm_samples" + ".bag", rosbag::bagmode::Append);
  wbag.open(l_gmm_dir + "_learned_gmm_joint_samples" + ".bag", rosbag::bagmode::Append);

  sensor_msgs::JointState sample;
  sample.header.frame_id = "panda_link0";
  sample.position = sample_pnt;

  wbag.write(("/joint_samples"), ros::Time::now(), sample);
  wbag.close();
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

// // doRegression for TPGMM -- regressing over time samples
// void doRegression(Eigen::MatrixXf  xi, const std::vector<Eigen::VectorXf>  means, const std::vector<float>  weights , const std::vector<Eigen::MatrixXf>  covariances)
// {
//   std::cout << "    *** Starting Regression ***    " << std::endl;
  
//   // int numTrajsamples = 1;     // number of points in the chosen trajectory
//   // int i = 0;
//   // std::string trajDir;

//   // rosbag::Bag rbag, wbag;

//   geometry_msgs::PoseArray learned_posesArray;
//   geometry_msgs::Pose pose;

//   Eigen::VectorXf X;
//   Eigen::VectorXf mean;
//   Eigen::MatrixXf covariance;

//   Eigen::MatrixXf out_xi;

//   // out_xi.resize(xi.rows() ,xi.cols());
//   // out_xi=Eigen::MatrixXf::Zero(xi.rows(), xi.cols());
//   // numTrajsamples = 703;
//   out_xi.resize(xi.rows(), xi.cols());
//   out_xi=Eigen::MatrixXf::Zero(xi.rows(), xi.cols());

//   // std::cout << "numTrajsamples:" << numTrajsamples << std::endl;
//   // std::cout<< "xi:" <<std::endl << xi.size()/4 <<std::endl;
//   // std::cout<< "means.size(): " << means.size() <<std::endl;
//   // std::cout<< "covariances: " << covariances.size() <<std::endl;
//   // std::cout<< "weights: " << weights.size() <<std::endl;

// //equation 10
//   float xi_hat_s_k, mu_s_k,mu_t_k,sigma_st_k,inv_sigma_t_k,sigma_hat_s_k,
//         sigma_s_k, sigma_t_k,sigma_ts_k, xi_t, beta_k,xi_hat_s,beta_k_sum, beta_k_xi_hat_s_k_sum;

//   for(std::size_t i = 0; i < out_xi.rows(); i++)   // loop over data points
//   {
//     for(std::size_t coord=1; coord<out_xi.cols(); coord++)    // loop over the xyz coordinates
//     {
//       beta_k_sum=0;
//       beta_k_xi_hat_s_k_sum=0;
//       for(std::size_t k=0; k<means.size(); k++)     // loop over the number of gaussians
//       {
//         mu_t_k=means.at(k)(0);
//         mu_s_k=means.at(k)(coord);
//         sigma_st_k=covariances.at(k)(coord,0);
//         sigma_t_k=covariances.at(k)(0,0);
//         inv_sigma_t_k=1/sigma_t_k;
//         sigma_s_k=covariances.at(k)(coord,coord);
//         sigma_ts_k=covariances.at(k)(0,coord);
//         // xi_t=xi(i,0);
//         xi_t = i;
//         // xi_t = sqrt(pow(mp->O_T_EE_c[12],2) + pow(mp->O_T_EE_c[13],2) + pow(mp->O_T_EE_c[14],2));
//         // xi_t = pose_regress;
//         // xi_t = xi(i,3);
//         xi_hat_s_k=mu_s_k+ sigma_st_k*inv_sigma_t_k*(xi_t -mu_t_k);
//         sigma_hat_s_k=sigma_s_k -sigma_st_k*inv_sigma_t_k*sigma_ts_k;


//         //equation 11

//         X.resize(1,1);
//         mean.resize(1,1);
//         covariance.resize(1,1);



//         X<<xi_t;
//         mean<<mu_t_k;

//         covariance<<sigma_t_k;




//         beta_k=weights.at(k) * computeNormalDistributionPDF(X, mean,covariance);
//         beta_k_sum=beta_k_sum+beta_k;

//         xi_hat_s=beta_k*xi_hat_s_k;
//         beta_k_xi_hat_s_k_sum=beta_k_xi_hat_s_k_sum+xi_hat_s;
//       }
//       out_xi(i,0)=xi_t;
//       out_xi(i,coord)=beta_k_xi_hat_s_k_sum/beta_k_sum;
//     }

//     pose.position.x = out_xi(i,1);
//     pose.position.y = out_xi(i,2);
//     pose.position.z = out_xi(i,3);
//     learned_posesArray.poses.push_back(pose);
//   }

//   learned_posesArray.header.frame_id = "panda_link0";
//   learned_posesArray_pub.publish(learned_posesArray);

//   // // Publish the learned_trajectory on topic /gmm_node/gmm/learned_trajectory
//   // learned_pose.header.frame_id = "panda_link0";
//   // learned_pose_pub.publish(learned_pose);
//   // ros::Duration(0.01).sleep();

//   std::cout<< "-------------------------------------" <<std::endl;

//   std::cout<< "out_xi:" <<std::endl <<out_xi <<std::endl;

//   std::cout<< "out_xi.rows():"  <<out_xi.rows() <<std::endl;
//   std::cout<< "out_xi.cols():"  <<out_xi.cols() <<std::endl;
// }

std::vector<float> doRegression(int input, Eigen::MatrixXf  xi, const std::vector<Eigen::VectorXf>  means, const std::vector<float>  weights , const std::vector<Eigen::MatrixXf>  covariances)
{
  std::cout << "    *** Starting Regression ***    " << std::endl;
  
  int numTrajsamples = 1;     // number of points in the chosen trajectory
  int i = 0;

  Eigen::VectorXf X;
  Eigen::VectorXf mean;
  Eigen::MatrixXf covariance;

  Eigen::MatrixXf out_xi;

  out_xi.resize(numTrajsamples, xi.cols());
  out_xi=Eigen::MatrixXf::Zero(numTrajsamples, xi.cols());

//equation 10
  float xi_hat_s_k, mu_s_k,mu_t_k,sigma_st_k,inv_sigma_t_k,sigma_hat_s_k,
        sigma_s_k, sigma_t_k,sigma_ts_k, xi_t, beta_k,xi_hat_s,beta_k_sum, beta_k_xi_hat_s_k_sum;

  for(std::size_t coord=1; coord<out_xi.cols(); coord++)    // loop over the xyz coordinates (dimenstions of the model)
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
      xi_t = input; //pose_regress;
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




      beta_k=weights.at(k) * computeNormalDistributionPDF(X, mean, covariance);
      beta_k_sum=beta_k_sum+beta_k;

      xi_hat_s=beta_k*xi_hat_s_k;
      beta_k_xi_hat_s_k_sum=beta_k_xi_hat_s_k_sum+xi_hat_s;
    }
    out_xi(0,0)=xi_t;
    out_xi(0,coord)=beta_k_xi_hat_s_k_sum/beta_k_sum;
  }
  
  std::cout<< "-------------------------------------" <<std::endl;

  std::cout<< "out_xi:" <<std::endl <<out_xi <<std::endl;


  std::cout<< "out_xi.rows():"  <<out_xi.rows() <<std::endl;
  std::cout<< "out_xi.cols():"  <<out_xi.cols() <<std::endl;

  return {out_xi(0,1), out_xi(0,2), out_xi(0,3)};
}

// THIS FUNCTION IS SUPPOSED TO BE IN model_based_planning_context.cpp, but I put it here as a backup
// ompl::base::ValidStateSamplerPtr
// ompl_interface::ModelBasedPlanningContext::allocGaussianSampler(const ompl::base::SpaceInformation* si) const
// {
//   // Getting the GMM from the .bag file
//   rosbag::Bag rbag;
//   rbag.open("/home/zizo/Disassembly Teleop/LL/Rcover/gmm-gmr/gmm_mix_action2.bag");

//   // Initializing Variables
//   Eigen::MatrixXf GMM_X;
//   std::vector<Eigen::VectorXf> GMM_means;
//   std::vector<Eigen::MatrixXf> GMM_covariances;
//   std::vector<float> GMM_weights;

//   for(rosbag::MessageInstance const m: rosbag::View(rbag))
//   {
//     gaussian_mixture_model::GaussianMixture::ConstPtr i = m.instantiate<gaussian_mixture_model::GaussianMixture>();
//     if (i != nullptr)
//     {
//       int GMM_dim = sizeof(i->gaussians[0].means)/3; // 3 cause it seems the size of float32 is 3 bytes instead of 4 bytes!!
//       ROS_INFO("GMM Dimension: %d", GMM_dim);

//       // Read GaussianMixutre msg and convert it to a format that doRegression(....) accepts
//       GMM_X = Eigen::MatrixXf::Identity(1,GMM_dim); // Not really necessary, it's just only to abide by doRegression(....) format
//       GMM_means.clear();
//       GMM_covariances.clear();
//       Eigen::VectorXf G_means(GMM_dim);
//       Eigen::MatrixXf G_covariances(GMM_dim, GMM_dim);      
//       GMM_weights = i->weights;
//       for (gaussian_mixture_model::Gaussian gaussian : i->gaussians)
//       {
//         // Eigen::VectorXf G_means(GMM_dim);
//         // Eigen::MatrixXf G_covariances(GMM_dim, GMM_dim);
//         int i = 0;
//         for (int d = 0; d < GMM_dim; d++)
//         {
//           G_means(d) = gaussian.means[d];

//           for (int c = 0; c < GMM_dim; c++)
//           {
//             G_covariances(d,c) = gaussian.covariances[i]; //[c+i]
//           }
//           i++; //=+ GMM_dim; // To make sure that each row from gaussian.coavariances is placed correctly in G_covariances
//         }
//         GMM_means.push_back(G_means);
//         GMM_covariances.push_back(G_covariances);
//       }
//     }
//   }
//   rbag.close();

//   //Debugging
//   std::cout << "Debugging parameters from .bag file" << std::endl;
//   // for (float w : GMM_weights) {std::cout << w << " ";}

//   // for (Eigen::VectorXf gaussian : GMM_means)
//   // {
//   //   std::cout << gaussian << std::endl;
//   //   // for (float mean : gaussian)
//   //   // {
//   //   //   std::cout << mean << " ";
//   //   // }
//   //   // std::endl;
//   // }
//   // std::cout << "-------" << std::endl;
//   //\Debugging

//   // std::cout << "StateSpace Name: " << si->getStateSpace()->getType() << std::endl;

//   // return std::make_shared<ompl::base::GaussianValidStateSampler>(si);
//   return std::make_shared<ompl::base::GMRValidStateSampler>(si, GMM_X, GMM_means, GMM_weights, GMM_covariances);
// }
  #endif