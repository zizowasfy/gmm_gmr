#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h> 
#include <franka_msgs/FrankaState.h>

#include <data_handle/DemonsInfo.h>

#include <boost/filesystem.hpp>
using namespace boost::filesystem;

class DemonsEntry
{
private:

  ros::ServiceServer service;
  ros::Publisher poseArray_pub;
  ros::Publisher poseArrayViz_pub;
  ros::Publisher poseViz_pub;

  rosbag::Bag rbag, wbag;

  std::string task_param;
  std::string subtask_param;

  struct {
    std::vector<std::string> names;
    std::vector<int> poses;
    std::vector<float> minRange, maxRange;
  } demonsinfo;

public:
  DemonsEntry(ros::NodeHandle n)
  {
    n.getParam("task_param", task_param);
    n.getParam("subtask_param", subtask_param);
    std::cout << "task_param: " << task_param.c_str() << " | subtask_param: " << subtask_param.c_str() << std::endl;

    service = n.advertiseService("DemonsInfo_Service", &DemonsEntry::demonsInfo, this);
    poseArray_pub = n.advertise<geometry_msgs::PoseArray>("/gmm/traj_input", 10);
    poseArrayViz_pub = n.advertise<geometry_msgs::PoseArray>("/traj_input_viz", 10);
    poseViz_pub = n.advertise<geometry_msgs::PoseStamped>("/pose_viz", 10);

    // readDemons();
  }

  bool demonsInfo(data_handle::DemonsInfo::Request &req, data_handle::DemonsInfo::Response &res)
  {
    ROS_INFO("DEMONSINFO Service has been Requested");
    
    if (req.read) {readDemons();};  // Do not read the Demonstrations directory unless read==true

    res.names = demonsinfo.names;
    res.numofposes = demonsinfo.poses;

    // Finding the index of the Longest and Shortest Demonstration
    auto it = std::max_element(demonsinfo.poses.begin(), demonsinfo.poses.end());
    if (it != demonsinfo.poses.end()) 
    {
      int idx = std::distance(demonsinfo.poses.begin(), it);
      std::cout << "Longest Demon index: " << idx << std::endl;
      res.longest_idx = idx;
    }

    it = std::min_element(demonsinfo.poses.begin(), demonsinfo.poses.end());
    if (it != demonsinfo.poses.end()) 
    {
      int idx = std::distance(demonsinfo.poses.begin(), it);
      std::cout << "Shortest Demon index: " << idx << std::endl;
      res.shortest_idx = idx;
    }
    //\ Finding the index of the Longest and Shortest Demonstration

    // Finding the smallest and biggest value in all Demons (define the range limits)
    res.minRange = demonsinfo.minRange;
    res.maxRange = demonsinfo.maxRange;
    //\ Finding the smallest and biggest value in all Demons (define the range limits)

    // Print the Response of the Service (Output)
    for (std::string i : demonsinfo.names) {std::cout << i << " ";}    
    std::cout << std::endl;

    for (int i : demonsinfo.poses) {std::cout << i << " ";}
    std::cout << std::endl;
    //\ Print the Response of the Service (Output)

    return true;
  }

  void readDemons()
  {
    // int i,j; i = j = 1; 
    demonsinfo.names.clear(); demonsinfo.poses.clear();

    std::string path = "/home/zizo/Disassembly Teleop/Demons/" + task_param + "/" + subtask_param;
    for (const auto& dirEntry : recursive_directory_iterator(path))
      {
        // If 'action' and 'place' demonstrations are in the one folder, this code snippet distinguishes them
        // std::cout << dirEntry << std::endl;
        // if (dirEntry.path().string().find("action") != std::string::npos) 
        // {
        //   std::cout << dirEntry << ": " << i << std::endl;   
        //   i++;
        // }
        // else if (dirEntry.path().string().find("place") != std::string::npos) 
        // {
        //   std::cout << dirEntry << ": " << j << std::endl;   
        //   j++;
        // }
        //\ If 'action' and 'place' demonstrations are in the one folder, this code snippet distinguishes them

        int msg_count = 0;
        rbag.open(dirEntry.path().string());
        for(rosbag::MessageInstance const m: rosbag::View(rbag))
        {
          franka_msgs::FrankaState::ConstPtr mp = m.instantiate<franka_msgs::FrankaState>();
          if (mp != nullptr)
          {
            // std::cout << mp->O_T_EE[14] << std::endl;
            msg_count++;
          }
        }


        // Search for second occurence of 'Demon' word in the directory to properly name the Demonstrations
        std::size_t D1 = dirEntry.path().string().find("Demon", 0, 5);
        std::size_t D2 = dirEntry.path().string().find("Demon", D1 + 1);
        std::size_t dot = dirEntry.path().string().find(".");
        // if (D2 != std::string::npos) {std::cout << D1 << " " << D2 << dot << std::endl;}
        //\ Search for second occurence of 'Demon' word in the directory to properly name the Demonstrations

        demonsinfo.names.push_back(dirEntry.path().string().substr(D2,dot-D2));
        demonsinfo.poses.push_back(msg_count);

        // // Printing for Debugging
        // std::cout << dirEntry << std::endl;
        // std::cout << dirEntry.path().string().substr(79,6) << ": " << msg_count << std::endl;
        // //\ Printing for Debugging

        rbag.close();
      }

    sendTrajPoses();
  }

  void sendTrajPoses()
  {
    std::vector<float> minError = {1000, 1000, 1000}; std::vector<float> maxError = {0, 0, 0}; 
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped poseViz;
    geometry_msgs::PoseArray poseArray;
    geometry_msgs::PoseArray poseArrayViz;

    poseArray.header.frame_id = "panda_link0";
    poseArrayViz.header.frame_id = "panda_link0";
        
    std::string path = "/home/zizo/Disassembly Teleop/Demons/" + task_param + "/" + subtask_param;
    for (const auto& dirEntry : recursive_directory_iterator(path))
    {
      int msg_count = 0;
      geometry_msgs::Pose prev_pose;
      rbag.open(dirEntry.path().string());
      for(rosbag::MessageInstance const m: rosbag::View(rbag))
      {
        franka_msgs::FrankaState::ConstPtr mp = m.instantiate<franka_msgs::FrankaState>();
        if (mp != nullptr)
        {
          // use when learning absolute poses
          pose.pose.position.x = msg_count == 0 ? 0 : (mp->O_T_EE[12]) * 1;
          pose.pose.position.y = msg_count == 0 ? 0 : (mp->O_T_EE[13]) * 1;
          pose.pose.position.z = msg_count == 0 ? 0 : (mp->O_T_EE[14]) * 1;
          // use when learning delta poses          
          // pose.pose.position.x = msg_count == 0 ? 0 : (mp->O_T_EE[12] - prev_pose.position.x) * 1;
          // pose.pose.position.y = msg_count == 0 ? 0 : (mp->O_T_EE[13] - prev_pose.position.y) * 1;
          // pose.pose.position.z = msg_count == 0 ? 0 : (mp->O_T_EE[14] - prev_pose.position.z) * 1;

          // Sample over (the x-axis) 
          pose.pose.orientation.w = sqrt(pow(mp->O_T_EE_c[12],2) + pow(mp->O_T_EE_c[13],2) + pow(mp->O_T_EE_c[14],2)); // euclidean distance of the delta position
          pose.pose.orientation.x = msg_count == 0 ? 0 : mp->O_T_EE_c[12];
          pose.pose.orientation.y = msg_count == 0 ? 0 : mp->O_T_EE_c[13];
          pose.pose.orientation.z = msg_count == 0 ? 0 : mp->O_T_EE_c[14];

          if (msg_count != 0) {poseArray.poses.push_back(pose.pose);} // for excluding the first point because the delta is zero.
          // ros::Duration(0.001).sleep();
          
          // // Finding the smallest and biggest value in all Demons (define the range limits)
          // double poseError[3] = {pose.orientation.x, pose.orientation.y, pose.orientation.z};
          // for (int i = 0; i < 3; i++)
          // {
          //   if (poseError[i] < minError[i]) {minError[i] = poseError[i];}
          //   if (poseError[i] > maxError[i]) {maxError[i] = poseError[i];}
          // }
          // //\ Finding the smallest and biggest value in all Demons (define the range limits)
  
          prev_pose.position.x = mp->O_T_EE[12];
          prev_pose.position.y = mp->O_T_EE[13];
          prev_pose.position.z = mp->O_T_EE[14];

          // Publish another poseArray but not the delta for visualization on Rviz (absolute poseArray)
          poseViz.pose.position.x = mp->O_T_EE[12];
          poseViz.pose.position.y = mp->O_T_EE[13];
          poseViz.pose.position.z = mp->O_T_EE[14];
          poseArrayViz.poses.push_back(poseViz.pose);
          //\ Publish another poseArray but not the delta for visualization on Rviz (absolute poseArray)
          
          // Finding the smallest and biggest value in all Demons (define the range limits)
          double poseError[3] = {poseViz.pose.position.x, poseViz.pose.position.y, poseViz.pose.position.z};
          for (int i = 0; i < 3; i++)
          {
            if (poseError[i] < minError[i]) {minError[i] = poseError[i];}
            if (poseError[i] > maxError[i]) {maxError[i] = poseError[i];}
          }
          //\ Finding the smallest and biggest value in all Demons (define the range limits)
  
          msg_count++;
          if (msg_count != 0) {poseViz_pub.publish(pose);}
        }
      }

      rbag.close();
    }
    ROS_INFO_STREAM("posesArray size: " << poseArray.poses.size());
    poseArray_pub.publish(poseArray);
    poseArrayViz_pub.publish(poseArrayViz);
    // poseViz.header.frame_id = "panda_link0";
    // poseViz_pub.publish(poseViz);
    demonsinfo.minRange = minError;
    demonsinfo.maxRange = maxError;
  }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "DemonsEntry");

  // std::cout << argc << std::endl;
  // std::cout << argv[1] << std::endl;

  ros::NodeHandle n;

  DemonsEntry demonsEntry(n);

  ros::spin();
  return 0;
}