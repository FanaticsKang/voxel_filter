#include "semantic_point.h"
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

  pcl::io::loadPCDFile(argv[1], *cloud_in);
  std::cerr << "PointCloud before filtering: " << cloud_in->size()
            << " data points (" << pcl::getFieldsList(*cloud_in) << " )"
            << std::endl;

  VoxelFilter<pcl::SemanticPoint> filter(cloud_in);
  filter.setLeafSize(0.1, 0.1, 0.1);
  filter.applyFilter(*cloud_filtered);

  std::cout << "filter: " << cloud_in->size() << " / " << cloud_filtered->size()
            << std::endl;
}