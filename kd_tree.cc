#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/kdtree/kdtree_flann.h>

#include "semantic_point.h"
#include "toc.h"
#include "voxel_filter.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "参数个数太少： ./demo_voxel_grid pcd" << std::endl;
    return -1;
  }
  pcl::PointCloud<pcl::SemanticPoint>::Ptr cloud_in(
      new pcl::PointCloud<pcl::SemanticPoint>);
  pcl::PointCloud<pcl::SemanticPoint>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::SemanticPoint>);

  pcl::PointCloud<pcl::SemanticPoint>::Ptr cloud_filtered_2(
      new pcl::PointCloud<pcl::SemanticPoint>);

  pcl::io::loadPCDFile(argv[1], *cloud_in);
  std::cerr << "PointCloud before filtering: " << cloud_in->size()
            << " data points (" << pcl::getFieldsList(*cloud_in) << " )"
            << std::endl;
  int index = 0;
  for (auto& tmp : cloud_in->points) {
    if (tmp.label == 8) {
      ++index;
      tmp.type = 5;
    }
  }
  pcl::KdTreeFLANN<pcl::SemanticPoint> kdtree;
  kdtree.setInputCloud(cloud_in);

}