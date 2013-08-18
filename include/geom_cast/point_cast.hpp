#ifndef POINT_CAST_HPP_EGWZH108
#define POINT_CAST_HPP_EGWZH108

#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/integral_constant.hpp>

#include <Eigen/Core>

#include <pcl/point_types.h>

#include <geometry_msgs/Point.h>

#include <tf/tf.h>

#include <opencv2/core/core.hpp>

namespace ca
{

///////////////////////////////////////////////////////////////////////////////
// point set/get traits

template<typename T>
struct xyz_member_getset : public boost::false_type { };

template<typename T>
struct xyz_array_getset : public boost::false_type { };

template<typename T>
struct xyz_ctor_set : public boost::false_type { };

template<typename T>
struct xyz_tfget_get : public boost::false_type { };

template<typename T>
struct xyz_array_get : public boost::false_type { };

///////////////////////////////////////////////////////////////////////////////
// trait implementations for various point types

// tf::Vector3
template<>
struct xyz_tfget_get<tf::Vector3> : public boost::true_type { };

template<>
struct xyz_ctor_set<tf::Vector3> : public boost::true_type { };

// pcl
template<>
struct xyz_member_getset<pcl::PointXYZ> : public boost::true_type { };

template<>
struct xyz_member_getset<pcl::PointXYZI> : public boost::true_type { };

template<>
struct xyz_member_getset<pcl::PointXYZRGB> : public boost::true_type { };

template<>
struct xyz_member_getset<pcl::PointXYZL> : public boost::true_type { };

template<>
struct xyz_member_getset<pcl::PointWithViewpoint> : public boost::true_type { };

// geometry_msgs
template<>
struct xyz_member_getset<geometry_msgs::Point> : public boost::true_type { };

template<>
struct xyz_member_getset<geometry_msgs::Vector3> : public boost::true_type { };

// eigen
template<class Scalar>
struct xyz_array_getset<Eigen::Matrix<Scalar, 3, 1> > : public boost::true_type { };

template<class Scalar>
struct xyz_array_getset<Eigen::Matrix<Scalar, 4, 1> > : public boost::true_type { };

template<class Scalar>
struct xyz_array_getset<Eigen::Map<const Eigen::Matrix<Scalar, 3, 1> > > : public boost::true_type { };

template<class Scalar>
struct xyz_array_getset<Eigen::Map<const Eigen::Matrix<Scalar, 4, 1> > > : public boost::true_type { };

// raw pointer. a bit dangerous since it matches any pointer.
// TODO restrict to numeric types. No boundary checks either.
template<class Scalar>
struct xyz_array_get<Scalar *> : public boost::true_type { };

// raw arrays. a bit dangerous since it matches any array.
// TODO restrict to numeric types.
template<class Scalar>
struct xyz_array_getset<Scalar [3]> : public boost::true_type { };

// opencv
template<class Scalar>
struct xyz_member_getset<cv::Point3_<Scalar> > : public boost::true_type { };

///////////////////////////////////////////////////////////////////////////////
// non-exhaustive set of converters for various src/target combinations.
// TODO add useful converters as needed.

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< xyz_member_getset<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< xyz_member_getset<Target> >::type* dummy2 = 0
                 ) {
  Target tgt;
  tgt.x = src.x;
  tgt.y = src.y;
  tgt.z = src.z;
  return tgt;
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< xyz_tfget_get<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< xyz_member_getset<Target> >::type* dummy2 = 0
                 ) {
  Target tgt;
  tgt.x = src.getX();
  tgt.y = src.getY();
  tgt.z = src.getZ();
  return tgt;
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< xyz_array_getset<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< xyz_array_getset<Target> >::type* dummy2 = 0
                 ) {
  Target tgt;
  tgt[0] = src[0];
  tgt[1] = src[1];
  tgt[2] = src[2];
  return tgt;
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< xyz_array_get<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< xyz_array_getset<Target> >::type* dummy2 = 0
                 ) {
  Target tgt;
  tgt[0] = src[0];
  tgt[1] = src[1];
  tgt[2] = src[2];
  return tgt;
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< xyz_array_getset<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< xyz_member_getset<Target> >::type* dummy2 = 0
                 ) {
  Target tgt;
  tgt.x = src[0];
  tgt.y = src[1];
  tgt.z = src[2];
  return tgt;
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< xyz_array_get<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< xyz_member_getset<Target> >::type* dummy2 = 0
                 ) {
  Target tgt;
  tgt.x = src[0];
  tgt.y = src[1];
  tgt.z = src[2];
  return tgt;
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< xyz_member_getset<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< xyz_array_getset<Target> >::type* dummy2 = 0
                 ) {
  Target tgt;
  tgt[0] = src.x;
  tgt[1] = src.y;
  tgt[2] = src.z;
  return tgt;
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< xyz_member_getset<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< xyz_ctor_set<Target> >::type* dummy2 = 0
                 ) {
  return Target(src.x, src.y, src.z);
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< xyz_tfget_get<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< xyz_array_getset<Target> >::type* dummy2 = 0
                 ) {
  Target tgt;
  tgt[0] = src.getX();
  tgt[1] = src.getY();
  tgt[2] = src.getZ();
  return tgt;
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< xyz_tfget_get<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< xyz_ctor_set<Target> >::type* dummy2 = 0
                 ) {
  return Target(src.getX(), src.getY(), src.getZ());
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< xyz_array_getset<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< xyz_ctor_set<Target> >::type* dummy2 = 0
                 ) {
  return Target(src[0], src[1], src[2]);
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< xyz_array_get<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< xyz_ctor_set<Target> >::type* dummy2 = 0
                 ) {
  return Target(src[0], src[1], src[2]);
}

#if 0
pcl::PointXYZ vector3f_to_point_xyz(Eigen::Vector3f ep) {
  pcl::PointXYZ p;
  p.getVector3fMap() = ep;
  return p;
}
#endif

} /* ca */

#endif /* end of include guard: POINT_CAST_HPP_EGWZH108 */
