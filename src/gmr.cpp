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

#include <gaussian_mixture_model/gmr.h>

  // Receiving TPGMM from tp_gmm node and put it in the format for doRegression()
  void GMR::onTPGMM(gaussian_mixture_model::GaussianMixturePtr data)
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

    geometry_msgs::PoseArray regressed_trajectory;
    regressed_trajectory.header.frame_id = "base_link";
    doRegression(Xi, tpgmm_means, tpgmm_weights, tpgmm_covariances, regressed_trajectory);
  }


  // this function computes PDF of X which has lenght of k
  float GMR::computeNormalDistributionPDF(Eigen::VectorXf X, Eigen::VectorXf mean, Eigen::MatrixXf covariance)
  {
      float nominator, denominator,tmp;


      Eigen::MatrixXf tmp_nominator_matrixXf;
      tmp_nominator_matrixXf=( X.transpose()-mean.transpose() )  * covariance.inverse() *  (X-mean) ;
      tmp=-0.5*tmp_nominator_matrixXf(0,0);
      nominator=std::exp (tmp);

      int k=X.rows();

      tmp=std::pow(2*M_PI,k) *covariance.determinant();
      denominator=std::pow( tmp, 0.5 );
      return  (nominator/denominator);
  }
  
 
  void GMR::doRegression(Eigen::MatrixXf&  xi, 
  const std::vector<Eigen::VectorXf>& means, 
  const std::vector<float>&  weights,
  const std::vector<Eigen::MatrixXf>& covariances,
  geometry_msgs::PoseArray& learned_posesArray)
  {
    std::cout << "    *** Starting Regression ***    " << std::endl;
    
    // int numTrajsamples = 1;     // number of points in the chosen trajectory
    // int i = 0;
    // std::string trajDir;

    // rosbag::Bag rbag, wbag;

    // geometry_msgs::PoseArrayPtr learned_posesArray;
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

    // learned_posesArray->header.frame_id = "base_link";
    // learned_posesArray_pub.publish(learned_posesArray);

    // // Publish the learned_trajectory on topic /gmm_node/gmm/learned_trajectory
    // learned_pose.header.frame_id = "panda_link0";
    // learned_pose_pub.publish(learned_pose);
    // ros::Duration(0.01).sleep();

    std::cout<< "-------------------------------------" <<std::endl;

    std::cout<< "out_xi:" <<std::endl <<out_xi <<std::endl;

    std::cout<< "out_xi.rows():"  <<out_xi.rows() <<std::endl;
    std::cout<< "out_xi.cols():"  <<out_xi.cols() <<std::endl;

    // return learned_posesArray;
  }