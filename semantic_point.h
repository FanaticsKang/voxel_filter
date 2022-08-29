#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/pcl_base.hpp>

namespace pcl {
// enforce SSE padding for correct memory alignment
struct EIGEN_ALIGN16 SemanticPoint {
  PCL_ADD_POINT4D;  // preferred way of adding a XYZ+padding
  PCL_ADD_RGB;
  uint32_t label;
  uint32_t type;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::SemanticPoint, (float, x, x)
                        (float, y, y)
                        (float, z, z)
                        (uint32_t, rgba, rgba)
                        (uint32_t, type, type)
                        (uint32_t, label, label))

