#ifndef POINT_CAST_HPP_EGWZH108
#define POINT_CAST_HPP_EGWZH108

#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/integral_constant.hpp>

#include <Eigen/Core>

#include <pcl/point_types.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>

#include <tf/tf.h>

#include <opencv2/core/core.hpp>

namespace ca
{

namespace detail
{
///////////////////////////////////////////////////////////////////////////////
// point get traits

// for pcl, geometry_msg
template<typename T>
struct xyz_member_get : public boost::false_type { };

// for eigen, arrays
template<typename T>
struct xyz_array_get : public boost::false_type { };

// for tf::vector3 (which is a btvector), eigen
template<typename T>
struct xyz_fun_member_get : public boost::false_type { };

///////////////////////////////////////////////////////////////////////////////
// point set traits
// for pcl, geometry_msg
template<typename T>
struct xyz_member_set : public boost::false_type { };

// for eigen, arrays
template<typename T>
struct xyz_array_set : public boost::false_type { };

// for tf::vector3 (which is a btvector)
template<typename T>
struct xyz_ctor_set : public boost::false_type { };

///////////////////////////////////////////////////////////////////////////////
// trait implementations for various point types

// tf::Vector3
template<>
struct xyz_fun_member_get<tf::Vector3> : public boost::true_type { };

template<>
struct xyz_ctor_set<tf::Vector3> : public boost::true_type { };

// pcl get
template<>
struct xyz_member_get<pcl::PointXYZ> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointXYZI> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointXYZRGB> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointXYZRGBA> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointNormal> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointXYZRGBNormal> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointXYZL> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointWithViewpoint> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointWithRange> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointWithScale> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointSurfel> : public boost::true_type { };

// pcl set
template<>
struct xyz_member_set<pcl::PointXYZ> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointXYZI> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointXYZRGB> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointXYZRGBA> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointNormal> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointXYZRGBNormal> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointXYZL> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointWithViewpoint> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointWithRange> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointWithScale> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointSurfel> : public boost::true_type { };

// geometry_msgs get
template<>
struct xyz_member_get<geometry_msgs::Point> : public boost::true_type { };

template<>
struct xyz_member_get<geometry_msgs::Point32> : public boost::true_type { };

template<>
struct xyz_member_get<geometry_msgs::Vector3> : public boost::true_type { };

// geometry_msgs set
template<>
struct xyz_member_set<geometry_msgs::Point> : public boost::true_type { };

template<>
struct xyz_member_set<geometry_msgs::Point32> : public boost::true_type { };

template<>
struct xyz_member_set<geometry_msgs::Vector3> : public boost::true_type { };

// eigen
template<class Scalar>
struct xyz_ctor_set<Eigen::Matrix<Scalar, 3, 1> > : public boost::true_type { };

template<class Scalar>
struct xyz_fun_member_get<Eigen::Matrix<Scalar, 3, 1> > : public boost::true_type { };

template<class Scalar>
struct xyz_ctor_set<Eigen::Matrix<Scalar, 4, 1> > : public boost::true_type { };

template<class Scalar>
struct xyz_fun_member_get<Eigen::Matrix<Scalar, 4, 1> > : public boost::true_type { };

template<class Scalar>
struct xyz_ctor_set<Eigen::Map<const Eigen::Matrix<Scalar, 3, 1> > > : public boost::true_type { };

template<class Scalar>
struct xyz_fun_member_get<Eigen::Map<const Eigen::Matrix<Scalar, 3, 1> > > : public boost::true_type { };

template<class Scalar>
struct xyz_ctor_set<Eigen::Map<const Eigen::Matrix<Scalar, 4, 1> > > : public boost::true_type { };

template<class Scalar>
struct xyz_fun_member_get<Eigen::Map<const Eigen::Matrix<Scalar, 4, 1> > > : public boost::true_type { };

// raw pointer. a bit dangerous since it matches any pointer.
// TODO restrict to numeric types. No boundary checks either.
template<class Scalar>
struct xyz_array_get<Scalar *> : public boost::true_type { };

// raw arrays. a bit dangerous since it matches any array.
// TODO restrict to numeric types.
template<class Scalar>
struct xyz_array_get<Scalar [3]> : public boost::true_type { };

template<class Scalar>
struct xyz_array_set<Scalar [3]> : public boost::true_type { };

// opencv
template<class Scalar>
struct xyz_member_get<cv::Point3_<Scalar> > : public boost::true_type { };

template<class Scalar>
struct xyz_member_set<cv::Point3_<Scalar> > : public boost::true_type { };


} // detail

///////////////////////////////////////////////////////////////////////////////
// possibly non-exhaustive set of converters for various src/target combinations.
// TODO add useful converters as needed.

// get
// 1 member_get
// 2 array_get
// 3 fun_member_get

// set
// 1 member_set
// 2 array_set
// 3 ctor_set

// rows: src, cols: tgt
//   1 2 3
// 1 . . .
// 2 . . .
// 3 . . .

// TODO this could probably be automated with macros or smarter metaprogramming

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< detail::xyz_member_get<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< detail::xyz_member_set<Target> >::type* dummy2 = 0
                 ) {
  Target tgt;
  tgt.x = src.x;
  tgt.y = src.y;
  tgt.z = src.z;
  return tgt;
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< detail::xyz_fun_member_get<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< detail::xyz_member_set<Target> >::type* dummy2 = 0
                 ) {
  Target tgt;
  tgt.x = src.x();
  tgt.y = src.y();
  tgt.z = src.z();
  return tgt;
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< detail::xyz_array_get<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< detail::xyz_array_set<Target> >::type* dummy2 = 0
                 ) {
  Target tgt;
  tgt[0] = src[0];
  tgt[1] = src[1];
  tgt[2] = src[2];
  return tgt;
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< detail::xyz_array_get<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< detail::xyz_member_set<Target> >::type* dummy2 = 0
                 ) {
  Target tgt;
  tgt.x = src[0];
  tgt.y = src[1];
  tgt.z = src[2];
  return tgt;
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< detail::xyz_member_get<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< detail::xyz_array_set<Target> >::type* dummy2 = 0
                 ) {
  Target tgt;
  tgt[0] = src.x;
  tgt[1] = src.y;
  tgt[2] = src.z;
  return tgt;
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< detail::xyz_member_get<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< detail::xyz_ctor_set<Target> >::type* dummy2 = 0
                 ) {
  return Target(src.x, src.y, src.z);
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< detail::xyz_fun_member_get<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< detail::xyz_array_set<Target> >::type* dummy2 = 0
                 ) {
  Target tgt;
  tgt[0] = src.x();
  tgt[1] = src.y();
  tgt[2] = src.z();
  return tgt;
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< detail::xyz_fun_member_get<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< detail::xyz_ctor_set<Target> >::type* dummy2 = 0
                 ) {
  return Target(src.x(), src.y(), src.z());
}

template<typename Target, typename Source>
Target point_cast(const Source& src,
                  typename boost::enable_if< detail::xyz_array_get<Source> >::type* dummy1 = 0,
                  typename boost::enable_if< detail::xyz_ctor_set<Target> >::type* dummy2 = 0
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
