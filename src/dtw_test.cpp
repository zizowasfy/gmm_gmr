//
// dtw_class.cpp
//
// Copyright (c) 2019 Charles Jekel
// MIT licensed
//


// #include <iostream>
// #include <vector>
// #include <DTW.hpp>
//
// int main ()
// {
//   double p = 2;  // the p-norm to use; 2.0 = euclidean, 1.0 = manhattan
//   std::vector<std::vector<double> > a = { {0, 2}, {1, 2}, {2, 2}, {3, 2}, {4, 2} };
//   std::vector<std::vector<double> > b = { {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0} , {5, 6} };
//
//
//   // Compute the DTW distance between a an b
//   // a[number_of_data_points][number_of_dimensions]
//   // b[number_of_data_points][number_of_dimensions]
//   // The number of dimensions between a and b must agree
//
//   // initialize the DTW object
//   DTW::DTW MyDtw (a, b, p);
//   // The distance is calculated and stored in MyDtw.distance
//   std::cout << "DTW distance: " << MyDtw.distance << std::endl;
//
//   // The pairwise distance between each point is stored in
//   // MyDtw.pairwise_vector
//   // The shape is MyDtw.pairwise_vector[a.size()][b.size()]
//   std::cout << "First pairwise distance: " << MyDtw.pairwise_vector[0][0] << std::endl;
//
//   // The DTW distance matrix is stored in MyDtw.dtw_vector
//   // The shape is MyDtw.dtw_vector[a.size()][b.size()]
//   std::cout << "The 1,1 value of the DTW distance matrix: ";
//   std::cout << MyDtw.dtw_vector[1][1] << std::endl;
//
//   // You can calculate the DTW alignment path with MyDtw.path()
//   // This retruns a 2D vector of the indexies through MyDtw.dtw_vector
//   // for alignment. This path goes from MyDtw.dtw_vector[0][0], to
//   // MyDtw.dtw_vector[i][j], up until MyDtw.dtw_vector[a.size()-1][b.size()-1]
//   std::vector<std::vector<int> > path;
//   path = MyDtw.path();
//   std::cout << "DTW Path: " << std::endl;
//   for (int i = 0; i < path.size(); ++i)
//     std::cout << "i=" << path[i][0] << " j=" << path[i][1] << std::endl;
//   return 0;
// }

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <onlineL_Params.hpp>
#include <my_iiwa_pkg/Numoftrial.h>

#include <sys/stat.h>
// #include <sys/types.h>

inline bool file_exists (const std::string& name)
{
  std::ifstream f(name.c_str());
  return f.good();
}

inline void file_copy (const std::string str1, const std::string str2)
{
  std::ifstream  src(str1, std::ios::binary);
  std::ofstream  dst(str2, std::ios::binary);

  dst << src.rdbuf();
}


int main(int argc, char **argv)
{
  // std::ifstream  src(std::string(DIR_LEARNEDTRAJ)+"learned-traj-peginhole-D8-2gauss.bag", std::ios::binary);
  // std::ofstream  dst(std::string(DIR_LEARNEDTRAJ)+"learned-traj_COPY.bag", std::ios::binary);
  // dst << src.rdbuf();
  //   file_copy(std::string(DIR_LEARNEDTRAJ)+"learned-traj-peginhole-D8-2gauss.bag", std::string(DIR_LEARNEDTRAJ)+"learned-traj_COPY.bag");
  //
  //   std::cout << file_exists(std::string(DIR_LEARNEDTRAJ)+"learned-traj-peginhole-D8-2gauss.bag") << std::endl;
  // bool ay;
  // std::cin >> ay;
  // std::cout << ay << std::endl;
  ////
  // char* char_arr;
  // std::string dir(std::string(CREATE_DIR_DATA));
  // char_arr = &dir[0];
  //
  // mkdir(char_arr, 0777);
  ////
  // rosbag::Bag wbag;
  // wbag.open(std::string(DIR_NEW_DEMONS)+"newDemonss"+"/test.bag", rosbag::bagmode::Write);
  // wbag.close();
  ////
  // ros::init(argc, argv,"ros_test_node");
  // ros::NodeHandle nh;
  // ros::ServiceClient numofTrial_client = nh.serviceClient<my_iiwa_pkg::Numoftrial>("/numofTrial");
  //
  // my_iiwa_pkg::Numoftrial srv;
  // numofTrial_client.call(srv);
  // if (numofTrial_client.call(srv))
  // {
  //   ROS_INFO("Sum: %ld", (long int)srv.response.numofTrial);
  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service add_two_ints");
  // }
  ////
  std::vector<int> v = {1,2,3};

  std::string str;
  for (int i : v) {str+=(std::to_string(i)+" ");}
  std::cout << "{" << str << "}" << std::endl;
}
