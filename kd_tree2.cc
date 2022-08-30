#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/impl/pcl_base.hpp"
#include "pcl/kdtree/impl/kdtree_flann.hpp" 
#include "pcl/search/impl/organized.hpp"
#include "pcl/surface/impl/convex_hull.hpp"
#include <iostream>
// #include "pch.h"
struct PointIXYZ {
  PCL_ADD_POINT4D;  //该struct有四个元素
  int index;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointIXYZ, (int, index, index)(float, x, x)(float, y, y)(float, z, z))

int main() {
  pcl::PointCloud<PointIXYZ> cloud;
  cloud.width = 5;
  cloud.height = 1;
  cloud.resize(cloud.width * cloud.height);
  for (size_t i = 0; i < cloud.size(); ++i) {
    cloud.points[i].index = i;
    cloud.points[i].x = i + 1;
    cloud.points[i].y = i + 2;
    cloud.points[i].z = i + 3;
  }
  std::cerr << "自定义原始点云：" << std::endl
            << "索引  "
            << "x  "
            << "y  "
            << "z" << std::endl;
  for (auto &m : cloud) {
    std::cerr

        << m.index << "  " << m.x << "  " << m.y << "  " << m.z << std::endl;
  }
  pcl::KdTreeFLANN<PointIXYZ> tree;

  tree.setInputCloud(cloud.makeShared());
  std::vector<int> indices(0, 0);
  std::vector<float> dists(0, 0.0);
  if (tree.nearestKSearch(cloud.points[0], 3, indices, dists) > 0) {
    std::cerr << "第一个点近邻搜索点云数量与坐标：" << std::endl
              << indices.size() << std::endl;
  }
  for (size_t i = 0; i < indices.size(); ++i) {
    std::cerr << cloud.points[indices[i]].index << "  "
              << cloud.points[indices[i]].x << "  "
              << cloud.points[indices[i]].y << "  "
              << cloud.points[indices[i]].z << std::endl;
  }

  std::cout << "Hello World!\n";
}
