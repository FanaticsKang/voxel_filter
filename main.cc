#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>

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

  Alpha::VoxelFilter<pcl::SemanticPoint> filter(cloud_in);
  filter.setLeafSize(0.1, 0.1, 0.1);
  Alpha::TicToc tic;
  filter.applyFilter(*cloud_filtered);
  std::cout << "time cost: " << tic.TocMicroseconds() << std::endl;

  pcl::VoxelGrid<pcl::SemanticPoint> filter2;
  filter2.setInputCloud(cloud_in);
  filter2.setLeafSize(0.1, 0.1, 0.1);

  tic.Tic();
  filter2.filter(*cloud_filtered_2);
  std::cout << "icp time cost: " << tic.TocMicroseconds() << std::endl;

  int index_type = 0;
  for (auto& tmp : cloud_filtered->points) {
    if (tmp.type == 5) {
      ++index_type;
    }
  }

  std::cout << "filter(origin / ours / pcl): " << cloud_in->size() << " / "
            << cloud_filtered->size() << " / " << cloud_filtered_2->size()
            << ", line points: " << index << " / " << index_type << std::endl;
}