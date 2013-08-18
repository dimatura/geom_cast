#include <iostream>
#include "geom_cast/point_cast.hpp"

int main(int argc, char *argv[]) {

  Eigen::Vector3d food(2, 8, 0);
  Eigen::Vector3f foof(2, 8, 0);

  tf::Vector3 v1 = ca::point_cast<tf::Vector3>(food);
  std::cerr << v1.x() << ", " << v1.y() << ", " << v1.z() << std::endl;

  tf::Vector3 v2 = ca::point_cast<tf::Vector3>(foof);
  std::cerr << v2.x() << ", " << v2.y() << ", " << v2.z() << std::endl;

  geometry_msgs::Point v3 = ca::point_cast<geometry_msgs::Point>(foof);
  std::cerr << v3.x << ", " << v3.y << ", " << v3.z << std::endl;

  geometry_msgs::Point v4 = ca::point_cast<geometry_msgs::Point>(v2);
  std::cerr << v4.x << ", " << v4.y << ", " << v4.z << std::endl;

  pcl::PointXYZ v5 = ca::point_cast<pcl::PointXYZ>(foof);
  std::cerr << v5.x << ", " << v5.y << ", " << v5.z << std::endl;

  Eigen::Vector3d v6 = ca::point_cast<Eigen::Vector3d>(v5);
  std::cerr << v6.x() << ", " << v6.y() << ", " << v6.z() << std::endl;

  Eigen::Vector3d v7 = ca::point_cast<Eigen::Vector3d>(v1);
  std::cerr << v7.x() << ", " << v7.y() << ", " << v7.z() << std::endl;

  return 0;
}
