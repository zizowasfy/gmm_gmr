#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>


#include <list>
#include <vector>
#include <DTW.hpp>

#include <onlineL_Params.hpp>
#include <my_iiwa_pkg/Numoftrial.h>
#include <my_iiwa_pkg/Numoftrialarr.h>

#include <sstream>
#include <iostream>

class Demons_DTW
{
private:
  std_msgs::Int32MultiArray numTrajsamples;
  ros::Subscriber numTrajsamples_sub;
  ros::Publisher posesArray_pub;
  ros::ServiceClient numofTrialArr_client;

  uint newDemons_count = 0;


  // void numTrajsamples_callback(std_msgs::Int32MultiArray msg)
  // {
  //   numTrajsamples = msg;
  //   // std::cout << numTrajsamples.data.size() << '\n';
  // }

  std::vector<std::vector<int> > RemoveRedundIndex(std::vector<std::vector<int> > path)
  {
    int prev_pnt = 1;
    std::vector<std::vector<int> > new_path;
    for (int i = 0; i < path.size(); i++)
    {
      new_path.push_back(path[i]);
      if (path[i][0] == prev_pnt)
      {
        new_path.pop_back();
      }
      prev_pnt = path[i][0];
    }
    return new_path;
  }

public:

  Demons_DTW(ros::NodeHandle n)
  {
    numTrajsamples_sub = n.subscribe("/GMM/numberofSamplesinDemos", 1, &Demons_DTW::newData, this);
    posesArray_pub = n.advertise<geometry_msgs::PoseArray>("/gmm/traj_input", 1000);
    numofTrialArr_client = n.serviceClient<my_iiwa_pkg::Numoftrialarr>("/numofTrialArr");
  }

  void newData(std_msgs::Int32MultiArray msg)
  {
    my_iiwa_pkg::Numoftrialarr srvarr;
    if (numofTrialArr_client.call(srvarr))
    { std::cout << "{ " ; for (int i : srvarr.response.numofTrialArr) { std::cout << i << " ," ; } std::cout << " }" << std::endl; }
    else { ROS_ERROR("Failed to call service numofTrialArr"); }

    newDemons_count++;
    numTrajsamples = msg;

    // _________ Reading Demonstrations data from demons bag files _________ //

    int maxindex = numTrajsamples.layout.data_offset;

    std::vector<std::vector<std::vector<double> > > demons_X;
    std::vector<std::vector<std::vector<double> > > demons_Y;
    std::vector<std::vector<std::vector<double> > > demons_Z;
    std::vector<std::vector<double> > demons_x;
    std::vector<std::vector<double> > demons_y;
    std::vector<std::vector<double> > demons_z;

    for (int D = 0; D < numTrajsamples.data.size(); D++)
    {
      rosbag::Bag bag;
      bag.open(std::string(DIR_DEMONS)+"Trial"+"_"+std::to_string(srvarr.response.numofTrialArr[D])+"_demon.bag", rosbag::bagmode::Read);

      std::vector<std::string> topics;
      topics.push_back(std::string("/Demonstration/CartesianPose/demonsPoses"));

      rosbag::View view(bag, rosbag::TopicQuery(topics));
      foreach(rosbag::MessageInstance const m, view)
      {
          geometry_msgs::PoseStamped::ConstPtr s = m.instantiate<geometry_msgs::PoseStamped>();
          if (s != NULL)
          {                   //       sampling       ,        xyz
            // demons_x.push_back({s->pose.orientation.y , s->pose.position.x});
            // demons_y.push_back({s->pose.orientation.y , s->pose.position.y});
            // demons_z.push_back({s->pose.orientation.y , s->pose.position.z});
            demons_x.push_back({s->pose.position.x});
            demons_y.push_back({s->pose.position.y});
            demons_z.push_back({s->pose.position.z});

            ros::Duration(0.002).sleep();
          }
      }
      demons_X.push_back(demons_x);
      demons_Y.push_back(demons_y);
      demons_Z.push_back(demons_z);
      demons_x.clear();
      demons_y.clear();
      demons_z.clear();

      bag.close();
    }


    // Printing all Demonstration Poses data
    // for (int i = 0; i < demons_X[0].size(); i++)
    // {
    //   std::cout << " | " << demons_x[i][0] << ' ' << demons_x[i][1] << ' ' << demons_y[i][1] << ' ' << demons_z[i][1] << " | " << '\n';
    // }
    // std::cout << demons_X[1][0][0] << '\n';
    // std::cout << demons_X.size() << '\n';
    // std::cout << demons_X[1].size() << '\n';
    // std::cout << demons_X[2].size() << '\n';
    // std::cout << demons_X[3].size() << '\n';
    // std::cout << demons_X[maxindex].size() << '\n';


    // \_________ Reading Demonstrations data from demons bag files _________ //


    // _________ Dynamic Time Warping DTW _________ //
    double p = 2.0;  // the p-norm to use; 2.0 = euclidean, 1.0 = manhattan
    std::vector<std::vector<int> > path_x, path_y, path_z;
    std::vector<std::vector<std::vector<int> > > path_X, path_Y, path_Z;
    std::vector<float> Dtw_XYZ;
    // std::vector<int> path_sizes;

    for (int D = 0; D < numTrajsamples.data.size(); D++)
    {
      // if (D == maxindex) { continue; }
      // initialize the DTW object
      DTW::DTW Dtw_X (demons_X[maxindex], demons_X[D], p);
      DTW::DTW Dtw_Y (demons_Y[maxindex], demons_Y[D], p);
      DTW::DTW Dtw_Z (demons_Z[maxindex], demons_Z[D], p);
      // The distance is calculated and stored in MyDtw.distance
      std::cout << "Dtw_X distance: " << Dtw_X.distance << std::endl;
      std::cout << "Dtw_Y distance: " << Dtw_Y.distance << std::endl;
      std::cout << "Dtw_Z distance: " << Dtw_Z.distance << std::endl;

      Dtw_XYZ.push_back(sqrt(pow(Dtw_X.distance,2) + pow(Dtw_Y.distance,2) + pow(Dtw_Z.distance,2)));
      std::cout << "Dtw_XYZ distance: " << Dtw_XYZ[D]  << std::endl;

      path_x = Dtw_X.path();
      path_y = Dtw_Y.path();
      path_z = Dtw_Z.path();

      // std::cout << "DTW Path: " << std::endl;
      // std::cout << path_x.size() << std::endl;
      // std::cout << path_y.size() << std::endl;
      // std::cout << path_z.size() << std::endl;
      // path_sizes.push_back(path_x.size()); path_sizes.push_back(path_y.size()); path_sizes.push_back(path_z.size());


      // This function makes the path_x, path_y, path_z equal in size (= to the size of the longest Demonstration)
      path_x = RemoveRedundIndex(path_x);
      path_y = RemoveRedundIndex(path_y);
      path_z = RemoveRedundIndex(path_z);

      // std::cout << "Post DTW Path: " << std::endl;
      // std::cout << path_x.size() << std::endl;
      // std::cout << path_y.size() << std::endl;
      // std::cout << path_z.size() << std::endl;

      path_X.push_back(path_x);
      path_Y.push_back(path_y);
      path_Z.push_back(path_z);
      // path_sizes.clear();

      // for (int i = 0; i < path_x.size(); ++i)
      //   {
      //     // std::cout << D+1 << ": xi=" << path_X[D][i][0] << " j=" << path_X[D][i][1] << std::endl;
      //     // std::cout << D+1 << ": yi=" << path_Y[D][i][0] << " j=" << path_Y[D][i][1] << std::endl;
      //     std::cout << D+1 << ": zi=" << path_Z[D][i][0] << " j=" << path_Z[D][i][1] << std::endl;
      //   }
    }

    // std::cout << "Post DTW total PATH: " << std::endl;
    // std::cout << path_X.size() << std::endl;
    // std::cout << path_Y.size() << std::endl;
    // std::cout << path_Z.size() << std::endl;

    // \_________ Dynamic Time Warping DTW _________ //

    // _________ Sending the Warped Trajectories _________ //

    geometry_msgs::PoseArray posesArray;
    geometry_msgs::Pose pose;
    geometry_msgs::PoseStamped demonPoses;
    rosbag::Bag wbag;

    posesArray.header.frame_id = "iiwa_link_0";
    demonPoses.header.frame_id = "iiwa_link_0";
    bool badD = false;
    for (int D = 0; D < numTrajsamples.data.size(); D++)    // numTrajsamples.data.size() must be equal to path_X.size() = no. of Demonstrations
    {
      wbag.open(std::string(DIR_DEMONS)+"Trial"+"_"+std::to_string(srvarr.response.numofTrialArr[D])+"_demon_DTW.bag", rosbag::bagmode::Write);
      if (Dtw_XYZ[D] > 1.5) { std::cout << "Demon " << D+1 << " is a Bad one" << std::endl; badD = true;}
      std::cout << "D: " << D+1 << '\n';

      for (int p = 0; p < demons_Z[maxindex].size(); p++)
      {
        // std::cout << demons_X[D][path_X[p][1]][0] << '\n';
        // std::cout << D << '\n';
        if (badD == false)    // == only send the not bad (good) Demonstrations
        {
          pose.position.x = demons_X[D][path_X[D][p][1]][0];
          pose.position.y = demons_Y[D][path_Y[D][p][1]][0];
          pose.position.z = demons_Z[D][path_Z[D][p][1]][0];
          pose.orientation.y = p;
          posesArray.poses.push_back(pose);
          ros::Duration(0.001).sleep();
        }

        demonPoses.pose.position.x = demons_X[D][path_X[D][p][1]][0];
        demonPoses.pose.position.y = demons_Y[D][path_Y[D][p][1]][0];
        demonPoses.pose.position.z = demons_Z[D][path_Z[D][p][1]][0];
        demonPoses.pose.orientation.y = p;
        // std::cout << "p: " << p << '\n';
        wbag.write("/Demonstration/CartesianPose/demonsPoses", ros::Time::now(), demonPoses);
        ros::Duration(0.002).sleep();
      }
      badD = false;
      wbag.close();
    }
    posesArray_pub.publish(posesArray);
    // ros::Duration(0.001).sleep();
  }
    // \_________ Sending the Warped Trajectories _________ //
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "DynamicTimeWarping");

  ros::NodeHandle n;

  Demons_DTW demons_DTW(n);

  ros::spin();
  return 0;
}
