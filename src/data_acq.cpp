
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "iiwa_msgs/CartesianPose.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>


#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <sstream>
#include <list>

#include <iostream>
#include <vector>
#include <DTW.hpp>

#include <onlineL_Params.hpp>
#include <my_iiwa_pkg/Numoftrial.h>
// using namespace std;

class Data_Acq
{
  private:

    // ____ Declaring variables ____ //
    geometry_msgs::PoseArray posesArray;
    geometry_msgs::Pose pose;
    geometry_msgs::PoseStamped demonPoses;
    ros::ServiceClient numofTrial_client;

    int newseq = 0;
    // std::list<int> npoints;
    std_msgs::Int32MultiArray DemosSamples;
    std_msgs::Int32 max, min;

    ros::Publisher numTrajsamples_pub;
    ros::Subscriber onlineLearning_sub;
    // \____ Declaring variables ____ //
  public:

  Data_Acq(ros::NodeHandle n)
  {
    numTrajsamples_pub = n.advertise<std_msgs::Int32MultiArray>("/GMM/numberofSamplesinDemos", 1);
    onlineLearning_sub = n.subscribe("/onlineLearning/Stage/", 1, &Data_Acq::newData, this);
    numofTrial_client = n.serviceClient<my_iiwa_pkg::Numoftrial>("/numofTrial");
  }

  void append_bags(rosbag::Bag & from, rosbag::Bag & to)
  {
    std::vector<std::string> topics;
    topics.push_back(std::string("/Demonstration/CartesianPose"));

    rosbag::View view(from, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
      geometry_msgs::PoseStamped::ConstPtr s = m.instantiate<geometry_msgs::PoseStamped>();
      if (s != NULL)
        to.write("/Demonstration/CartesianPose", ros::Time::now(), *s);
        // ros::Duration(0.001).sleep();
    }

    from.close();
    to.close();
  }

  void newData(const std_msgs::Int32 & onLstage) // NOTE: const is important here
  {
    if (onLstage.data == START_DATA_ACQ)
    {
      my_iiwa_pkg::Numoftrial srv;
      if (numofTrial_client.call(srv)) { ROS_INFO("numofTrial: %ld", (long int)srv.response.numofTrial); }
      else { ROS_ERROR("Failed to call service numofTrial"); }

            // ________________________________________ //
      rosbag::Bag rbag, wbag, newDbag, allDbag;
      uint numD = 1;
      newDbag.open(std::string(DIR_NEW_DEMONS)+"newDemons.bag", rosbag::bagmode::Read);

      allDbag.open(std::string(DIR_ALL_DEMONS)+"allDemons.bag", rosbag::bagmode::Append);

      std::cout << " " << std::endl;

      append_bags(newDbag, allDbag);

      rbag.open(std::string(DIR_ALL_DEMONS)+"allDemons.bag", rosbag::bagmode::Read);
      std::vector<std::string> topics;
      topics.push_back(std::string("/Demonstration/CartesianPose"));

      rosbag::View view(rbag, rosbag::TopicQuery(topics));
      int i = 0 ;
      foreach(rosbag::MessageInstance const m, view)
      {
          geometry_msgs::PoseStamped::ConstPtr s = m.instantiate<geometry_msgs::PoseStamped>();

          if (s != NULL)
              if (s->header.frame_id == "NEW_DEMON")
              {
                newseq = 1;// newseq = s->header.seq;
                // If there is more than one demonstration in a trial, it saves each one of them in a separate bag file under /newDemons(newDemons_count) folder
                wbag.close();
                wbag.open(std::string(DIR_NEW_DEMONS)+"newDemons"+std::to_string((long int)srv.response.numofTrial)+"_"+std::to_string(numD)+".bag", rosbag::bagmode::Write);
              }
              else if (s->header.frame_id == "DEMON_END")
              {
                // npoints.push_back(s->header.seq - newseq);
                DemosSamples.data.push_back(newseq); //DemosSamples.data.push_back(s->header.seq - newseq + 1); // + 1 VERY IMPORTANT to add to avoid misarrangement of samples of Demonstrations
                numD++;
              }
              // pose = s->pose;
              // posesArray.header.frame_id = "iiwa_link_0";
              // posesArray.poses.push_back(pose);
              // // posesArray_pub.publish(posesArray);
              // ros::Duration(0.001).sleep();
              // // ROS_INFO_STREAM(posesArray);
              demonPoses.header.frame_id = "iiwa_link_0";
              demonPoses.pose.orientation.y = s->header.seq - newseq + 1;
              demonPoses.pose.position.x = s->pose.position.x;
              demonPoses.pose.position.y = s->pose.position.y;
              demonPoses.pose.position.z = s->pose.position.z;

              wbag.write("/Demonstration/CartesianPose/demonsPoses", ros::Time::now(), demonPoses);
              ros::Duration(0.001).sleep();
              i++;
              newseq++;
      }

      std::cout << i << std::endl;
      rbag.close();

      std::cout << "Number of Demonstrations: "<< DemosSamples.data.size() << std::endl;

      max.data = 0; min.data = 100000;
      int maxindex = 0; int minindex = 0;
      for (int i = 0; i < DemosSamples.data.size(); i++)
      {
        if (DemosSamples.data[i] > max.data) {max.data = DemosSamples.data[i]; maxindex = i;}
        if (DemosSamples.data[i] < min.data) {min.data = DemosSamples.data[i]; minindex = i;}

        std::cout << DemosSamples.data[i] << ", ";
      }

      DemosSamples.layout.data_offset = minindex; //maxindex; //max.data; // THis carries the number of samples of the longest OR shortest Demonstrations

      std::cout << "max: " << max.data;
      std::cout << " min: " << min.data;
      std::cout << " maxindex: " << maxindex+1;
      std::cout << " minindex: " << minindex+1 << std::endl;

      std::cout << "Number of Total Poses: "<< posesArray.poses.size() << std::endl;

      // posesArray_pub.publish(posesArray);
      numTrajsamples_pub.publish(DemosSamples);
      DemosSamples.data.clear();
      std::cout << "Poses are now Published" << std::endl;
      std::cout << "\n";

      // // _________ Reading Demonstrations data from demons bag file
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ReadingDemonstrationsData");

  ros::NodeHandle n;

  Data_Acq data_acq(n);

  ros::spin();
  return 0;
}
