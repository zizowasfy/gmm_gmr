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

#include "trajectory_converter.h"

// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

// STL
#include <vector>
#include <string>
#include <sstream>

#define FORMAT_INDEX "index"
#define FORMAT_X     "x"
#define FORMAT_Y     "y"
#define FORMAT_Z     "z"
#define FORMAT_DX    "dx"
#define FORMAT_DY    "dy"
#define FORMAT_DZ    "dz"
#define COUNT_FORMAT 7

geometry_msgs::PoseStamped demonPoses;

static const char * const FORMAT_ALL[COUNT_FORMAT] =
  {
    FORMAT_INDEX,
    FORMAT_X,
    FORMAT_Y,
    FORMAT_Z,
    FORMAT_DX,
    FORMAT_DY,
    FORMAT_DZ,
  };

class FormatString
{
  public:
  typedef unsigned int uint;
  typedef boost::shared_ptr<FormatString> Ptr;

  FormatString(const std::string & f)
  {
    m_is_valid = true;

    // extract the parameters
    std::istringstream istr(f);
    std::string str;
    while (istr >> str)
      if (!str.empty())
        m_pars.push_back(str);

    // check the parameters
    for (uint i = 0; i < m_pars.size(); i++)
    {
      bool found = false;
      for (uint h = 0; h < COUNT_FORMAT; h++)
        if (FORMAT_ALL[h] == m_pars[i])
        {
          found = true;
          break;
        }

      if (!found)
      {
        ROS_WARN("trajectory_converter: invalid format parameter: \"%s\"",m_pars[i].c_str());
        m_is_valid = false;
      }
    }

    if (m_pars.size() == 0)
      m_is_valid = false;
  }

  uint getParameterCount() const
  {
    return m_pars.size();
  }

  bool isValid() const
  {
    return m_is_valid;
  }

  std::vector<float> format(const geometry_msgs::PoseArray & poses,uint index) const // , uint inum
  {
    std::vector<float> result;
    result.reserve(m_pars.size());

    uint ip = index < (poses.poses.size() - 1) ? index + 1 : index;
    uint im = index > 0 ? index - 1 : index;

    for (uint i = 0; i < m_pars.size(); i++)
    {
      if (m_pars[i] == FORMAT_INDEX)
        // result.push_back(float(index));
        // result.push_back(float(inum));
        result.push_back(float(poses.poses[index].orientation.y));
      else if (m_pars[i] == FORMAT_X)
        result.push_back(poses.poses[index].position.x);
      else if (m_pars[i] == FORMAT_Y)
        result.push_back(poses.poses[index].position.y);
      else if (m_pars[i] == FORMAT_Z)
        result.push_back(poses.poses[index].position.z);
      else if (m_pars[i] == FORMAT_DX)
        // result.push_back(float(inum));
        result.push_back(poses.poses[ip].position.x - poses.poses[im].position.x);
      else if (m_pars[i] == FORMAT_DY)
        result.push_back(poses.poses[ip].position.y - poses.poses[im].position.y);
      else if (m_pars[i] == FORMAT_DZ)
        result.push_back(poses.poses[ip].position.z - poses.poses[im].position.z);
    }

    // demonPoses.pose.orientation.y = inum;
    // demonPoses.pose.position.x = poses.poses[index].position.x;
    // demonPoses.pose.position.y = poses.poses[index].position.y;
    // demonPoses.pose.position.z = poses.poses[index].position.z;

    return result;
  }

  private:
  std::vector<std::string> m_pars;
  bool m_is_valid;
};

class TrajectoryConverter
{
  public:
  typedef unsigned int uint;

  TrajectoryConverter(ros::NodeHandle & nh): m_nh(nh)
  {
    std::string temp_string;

    m_nh.param<std::string>(PARAM_NAME_INPUT_TRAJ_TOPIC,temp_string,PARAM_DEFAULT_INPUT_TRAJ_TOPIC);
    m_input = m_nh.subscribe(temp_string,5,&TrajectoryConverter::onNewInput,this);

    numTrajsamples_sub = m_nh.subscribe("/GMM/numberofSamplesinDemos", 1, &TrajectoryConverter::numTrajsamples_callback, this);
    // creating publisher for publishing the poses of all demonstrations for plotting and other uses // NOT NEEDED ANYMORE
    // demonPoses_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/Demonstration/demonsPoses",100);

    m_nh.param<std::string>(PARAM_NAME_OUTPUT_DATA_TOPIC,temp_string,PARAM_DEFAULT_OUTPUT_DATA_TOPIC);
    m_output = m_nh.advertise<std_msgs::Float32MultiArray>(temp_string,5);

    m_nh.param<std::string>(PARAM_NAME_OUTPUT_FIELDS,temp_string,PARAM_DEFAULT_OUTPUT_FIELDS);
    m_format_string = FormatString::Ptr(new FormatString(temp_string));
    if (!m_format_string->isValid())
    {
      ROS_ERROR("trajectory_converter: format string \"%s\" is not valid!",temp_string.c_str());
      m_format_string = FormatString::Ptr(new FormatString(PARAM_DEFAULT_OUTPUT_FIELDS));
    }
  }

  void onNewInput(const geometry_msgs::PoseArray & poses)
  {
    std_msgs::Float32MultiArrayPtr out(new std_msgs::Float32MultiArray);

    uint ndim = m_format_string->getParameterCount();

    out->layout.dim.resize(2);
    out->layout.dim[1].size = ndim;
    out->layout.dim[0].size = poses.poses.size();

    out->data.resize(poses.poses.size() * ndim);

    // std::cout << poses.poses.size() << std::endl;

    for (uint i = 0; i < poses.poses.size(); i++)
    {
      std::vector<float> pars = m_format_string->format(poses,i);
      for (uint h = 0; h < ndim; h++)
      {
        out->data[i * ndim + h] = pars[h];
      }
    }
    // // Added by Zizo Shaarawy
    // // This code snippet arranges the demonstrations data so that they can be sent to the gmm_node PoseArray.
    // // This code snippet numbers each demon 0 -> N
    // rosbag::Bag bag; uint numD = 1;
    // uint i = 0;
    // uint prevD = 0;
    // uint inum = 0;
    //
    // for (uint D : numTrajsamples.data)
    // {
    //   bag.open("/home/mo/robotL_ws/src/record_data/GMM_data/Demons_Poses/demon"+std::to_string(numD)+".bag", rosbag::bagmode::Write);
    //
    //   inum = 0;
    //   for (i; i < D+prevD; i++)
    //   {
    //     std::vector<float> pars = m_format_string->format(poses,i,inum);
    //     // demonPoses_pub.publish(demonPoses);
    //     bag.write("/Demonstration/demonsPoses", ros::Time::now(), demonPoses);
    //     // ros::Duration(0.001).sleep();
    //     for (uint h = 0; h < ndim; h++)
    //     {
    //       out->data[i * ndim + h] = pars[h];
    //     }
    //     // std::cout << inum << std::endl;
    //     inum++;
    //   }
    //   // std::cout << "D:" << D << std::endl;
    //   prevD = D+prevD;
    //   numD++;
    //   bag.close();
    // }
    // // \ Added by Zizo Shaarawy

    m_output.publish(out);
  }

  private:
  ros::NodeHandle & m_nh;

  ros::Subscriber m_input;
  ros::Publisher m_output;

  ros::Subscriber numTrajsamples_sub;
  // ros::Publisher demonPoses_pub;
  std_msgs::Int32MultiArray numTrajsamples;


  void numTrajsamples_callback(std_msgs::Int32MultiArray msg)
  {
    numTrajsamples = msg;
  }

  FormatString::Ptr m_format_string;
};

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"trajectory_converter");

  ros::NodeHandle nh("~");
  TrajectoryConverter conv(nh);

  ros::spin();

  return 0;
}
