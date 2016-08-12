/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011, 2012 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: data_base.h 1554 2011-06-14 22:11:17Z jack.oquin $
 */

/** \file
 *
 *  Point Cloud Library point structures for Velodyne data.
 *
 *  @author Jesse Vera
 *  @author Jack O'Quin
 *  @author Piyush Khandelwal
 *  @author Alexander Schaefer
 */

#ifndef __VELODYNE_POINTCLOUD_POINT_TYPES_H
#define __VELODYNE_POINTCLOUD_POINT_TYPES_H

#include <pcl/point_types.h>

namespace velodyne_pointcloud
{
  /** Euclidean Velodyne coordinate, including intensity and ring number. */
  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

  // Euclidean coordinate, spherical coordinate, intensity and ring number.
  struct SphericalPoint
  {
      float sensor_x, sensor_y, sensor_z;
      float sensor_qw, sensor_qx, sensor_qy, sensor_qz;
      float azimuth, elevation, radius;
      float intensity;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;

  // Shorthand typedefs for point cloud representations
  typedef PointXYZIR VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;
  typedef SphericalPoint SPoint;
  typedef pcl::PointCloud<SPoint> SPointCloud;

}; // namespace velodyne_pointcloud


POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::SphericalPoint,
                                  (float, sensor_x, sensor_x)
                                  (float, sensor_y, sensor_y)
                                  (float, sensor_z, sensor_z)
                                  (float, sensor_qw, sensor_qw)
                                  (float, sensor_qx, sensor_qx)
                                  (float, sensor_qy, sensor_qy)
                                  (float, sensor_qz, sensor_qz)
                                  (float, azimuth, azimuth)
                                  (float, elevation, elevation)
                                  (float, radius, radius)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))

#endif // __VELODYNE_POINTCLOUD_POINT_TYPES_H
