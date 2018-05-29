#ifndef MULTI_LIDAR_CALIB_H
#define MULTI_LIDAR_CALIB_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/registration/icp.h>
// #include <pcl/registration/gicp.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/impl/pcl_base.hpp>
// #include <pcl/kdtree/impl/kdtree_flann.hpp>
// #include <pcl/search/impl/kdtree.hpp>
// #include <pcl/search/impl/organized.hpp>
// #include <pcl/features/impl/normal_3d.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_conversions/pcl_conversions.h>

#include "loam_velodyne/Twist.h"
#include "loam_velodyne/nanoflann_pcl.h"

const float PI = std::atan(1)*4;

namespace MSY_Denso{

/** Point label options. */
enum PointLabel {
CORNER_SHARP = 2,       ///< sharp corner point
CORNER_LESS_SHARP = 1,  ///< less sharp corner point
SURFACE_LESS_FLAT = 0,  ///< less flat surface point
SURFACE_FLAT = -1       ///< flat surface point
};


/** \brief Convert the given radian angle to degrees.
 *
 * @param radians The radian angle to convert.
 * @return The angle in degrees.
 */
inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}



/** \brief Convert the given radian angle to degrees.
 *
 * @param radians The radian angle to convert.
 * @return The angle in degrees.
 */
inline float rad2deg(float radians)
{
  return (float) (radians * 180.0 / M_PI);
}



/** \brief Convert the given degree angle to radian.
 *
 * @param degrees The degree angle to convert.
 * @return The radian angle.
 */
inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}



/** \brief Convert the given degree angle to radian.
 *
 * @param degrees The degree angle to convert.
 * @return The radian angle.
 */
inline float deg2rad(float degrees)
{
  return (float) (degrees * M_PI / 180.0);
}




/** \brief Calculate the squared difference of the given two points.
 *
 * @param a The first point.
 * @param b The second point.
 * @return The squared difference between point a and b.
 */
template <typename PointT>
inline float calcSquaredDiff(const PointT& a, const PointT& b)
{
  float diffX = a.x - b.x;
  float diffY = a.y - b.y;
  float diffZ = a.z - b.z;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}



/** \brief Calculate the squared difference of the given two points.
 *
 * @param a The first point.
 * @param b The second point.
 * @param wb The weighting factor for the SECOND point.
 * @return The squared difference between point a and b.
 */
template <typename PointT>
inline float calcSquaredDiff(const PointT& a, const PointT& b, const float& wb)
{
  float diffX = a.x - b.x * wb;
  float diffY = a.y - b.y * wb;
  float diffZ = a.z - b.z * wb;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}


/** \brief Calculate the absolute distance of the point to the origin.
 *
 * @param p The point.
 * @return The distance to the point.
 */
template <typename PointT>
inline float calcPointDistance(const PointT& p)
{
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}



/** \brief Calculate the squared distance of the point to the origin.
 *
 * @param p The point.
 * @return The squared distance to the point.
 */
template <typename PointT>
inline float calcSquaredPointDistance(const PointT& p)
{
  return p.x * p.x + p.y * p.y + p.z * p.z;
}



/** \brief Rotate the given vector by the specified angle around the x-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotX(Vector3& v, const Angle& ang)
{
  float y = v.y();
  v.y() = ang.cos() * y - ang.sin() * v.z();
  v.z() = ang.sin() * y + ang.cos() * v.z();
}

/** \brief Rotate the given point by the specified angle around the x-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
template <typename PointT>
inline void rotX(PointT& p, const Angle& ang)
{
  float y = p.y;
  p.y = ang.cos() * y - ang.sin() * p.z;
  p.z = ang.sin() * y + ang.cos() * p.z;
}



/** \brief Rotate the given vector by the specified angle around the y-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotY(Vector3& v, const Angle& ang)
{
  float x = v.x();
  v.x() = ang.cos() * x + ang.sin() * v.z();
  v.z() = ang.cos() * v.z() - ang.sin() * x;
}

/** \brief Rotate the given point by the specified angle around the y-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
template <typename PointT>
inline void rotY(PointT& p, const Angle& ang)
{
  float x = p.x;
  p.x = ang.cos() * x + ang.sin() * p.z;
  p.z = ang.cos() * p.z - ang.sin() * x;
}



/** \brief Rotate the given vector by the specified angle around the z-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotZ(Vector3& v, const Angle& ang)
{
  float x = v.x();
  v.x() = ang.cos() * x - ang.sin() * v.y();
  v.y() = ang.sin() * x + ang.cos() * v.y();
}

/** \brief Rotate the given point by the specified angle around the z-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
template <typename PointT>
inline void rotZ(PointT& p, const Angle& ang)
{
  float x = p.x;
  p.x = ang.cos() * x - ang.sin() * p.y;
  p.y = ang.sin() * x + ang.cos() * p.y;
}



/** \brief Rotate the given vector by the specified angles around the z-, x- respectively y-axis.
 *
 * @param v the vector to rotate
 * @param angZ the rotation angle around the z-axis
 * @param angX the rotation angle around the x-axis
 * @param angY the rotation angle around the y-axis
 */
inline void rotateZXY(Vector3& v,
                      const Angle& angZ,
                      const Angle& angX,
                      const Angle& angY)
{
  rotZ(v, angZ);
  rotX(v, angX);
  rotY(v, angY);
}

/** \brief Rotate the given point by the specified angles around the z-, x- respectively y-axis.
 *
 * @param p the point to rotate
 * @param angZ the rotation angle around the z-axis
 * @param angX the rotation angle around the x-axis
 * @param angY the rotation angle around the y-axis
 */
template <typename PointT>
inline void rotateZXY(PointT& p,
                      const Angle& angZ,
                      const Angle& angX,
                      const Angle& angY)
{
  rotZ(p, angZ);
  rotX(p, angX);
  rotY(p, angY);
}



/** \brief Rotate the given vector by the specified angles around the y-, x- respectively z-axis.
 *
 * @param v the vector to rotate
 * @param angY the rotation angle around the y-axis
 * @param angX the rotation angle around the x-axis
 * @param angZ the rotation angle around the z-axis
 */
inline void rotateYXZ(Vector3& v,
                      const Angle& angY,
                      const Angle& angX,
                      const Angle& angZ)
{
  rotY(v, angY);
  rotX(v, angX);
  rotZ(v, angZ);
}

/** \brief Rotate the given point by the specified angles around the y-, x- respectively z-axis.
 *
 * @param p the point to rotate
 * @param angY the rotation angle around the y-axis
 * @param angX the rotation angle around the x-axis
 * @param angZ the rotation angle around the z-axis
 */
template <typename PointT>
inline void rotateYXZ(PointT& p,
                      const Angle& angY,
                      const Angle& angX,
                      const Angle& angZ)
{
  rotY(p, angY);
  rotX(p, angX);
  rotZ(p, angZ);
}


class msy_multi_lidar{
public:

	explicit msy_multi_lidar();

void lidar_read1( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg);
void lidar_read2( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg);
void lidar_read3( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg);
void lidar_read4( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg);
void lidar_read5( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg);
void Registration(  const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn,
	                const ros::Time& scanTime,
	                pcl::PointCloud<pcl::PointXYZI> & PC_feature_corner,
	                pcl::PointCloud<pcl::PointXYZI> & PC_feature_surf);

void LM( pcl::PointCloud<pcl::PointXYZI> & Target_corner,
	  	 pcl::PointCloud<pcl::PointXYZI> & Target_surf,
	  	 pcl::PointCloud<pcl::PointXYZI> & Source_corner,
	  	 pcl::PointCloud<pcl::PointXYZI> & Source_surf);

int getRingForAngle(const float& angle);
void setScanBuffersFor(const size_t& startIdx, const size_t& endIdx, pcl::PointCloud<pcl::PointXYZI> & _laserCloud);
void setRegionBuffersFor(const size_t& startIdx, const size_t& endIdx, pcl::PointCloud<pcl::PointXYZI> & _laserCloud);

void transformTowardTarget(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
void FinalTransform(pcl::PointCloud<pcl::PointXYZI>& cloud);
void markAsPicked(const size_t& cloudIdx, const size_t& scanIdx, pcl::PointCloud<pcl::PointXYZI> & _laserCloud);

void setup(ros::NodeHandle& nh);
void spin();
bool process();

	// pcl::PointCloud<pcl::PointXYZI>::Ptr CloudFR_pt;
	// pcl::PointCloud<pcl::PointXYZI>::Ptr CloudFL_pt;
	// pcl::PointCloud<pcl::PointXYZI>::Ptr CloudTL_pt;
	// pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTC_pt;
	// pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTR_pt;

	pcl::PointCloud<pcl::PointXYZ> CloudTC_ni;
	pcl::PointCloud<pcl::PointXYZ> CloudTR_ni;

    pcl::PointCloud<pcl::PointXYZI>::Ptr CloudFR_sel;
    pcl::PointCloud<pcl::PointXYZI>::Ptr CloudFL_sel;
    pcl::PointCloud<pcl::PointXYZI>::Ptr CloudTL_sel;
    pcl::PointCloud<pcl::PointXYZI>::Ptr CloudTC_sel;
    pcl::PointCloud<pcl::PointXYZI>::Ptr CloudTR_sel;

	pcl::PointCloud<pcl::PointXYZI> CloudFR;
	pcl::PointCloud<pcl::PointXYZI> CloudFL;
	pcl::PointCloud<pcl::PointXYZI> CloudTL;
	pcl::PointCloud<pcl::PointXYZI> CloudTC;
	pcl::PointCloud<pcl::PointXYZI> CloudTR;

	int ringNum;
	float lowerBound;
	float upperBound;
	uint16_t nScanRings;
	float factor;

	float scanPeriod ;
	int imuHistorySize ;
	int nFeatureRegions ;
	int curvatureRegion ;
	int maxCornerSharp ;
	int maxCornerLessSharp ;
	int maxSurfaceFlat ;
	float lessFlatFilterSize ;
	float surfaceCurvatureThreshold ;

  	std::vector<PointLabel> _regionLabel;     ///< point label buffer
  	std::vector<int> _scanNeighborPicked;     ///< flag if neighboring point was already picked
	std::vector<float> _regionCurvature;      ///< point curvature buffer
  	std::vector<size_t> _regionSortIndices;   ///< sorted region indices based on point curvature

  	float transform_rx;
  	float transform_ry;
  	float transform_rz;
  	float transform_tx;
  	float transform_ty;
  	float transform_tz;

  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudOri;      ///< point selection
  pcl::PointCloud<pcl::PointXYZI>::Ptr _coeffSel;           ///< point selection coefficients

  Twist _transform;     ///< optimized pose transformation





private:
	ros::Subscriber sub1;
	ros::Publisher pub1;
	ros::Subscriber sub2;
	ros::Publisher pub2;
	ros::Subscriber sub3;
	ros::Publisher pub3;
	ros::Subscriber sub4;
	ros::Publisher pub4;
	ros::Subscriber sub5;
	ros::Publisher pub5;

	ros::Time _timeFR;
	ros::Time _timeFL;
	ros::Time _timeTL;
	ros::Time _timeTC;
	ros::Time _timeTR;

	Eigen::Matrix4f transformFR2TC;
	Eigen::Matrix4f transformFL2TC;
	Eigen::Matrix4f transformTL2TC;
	Eigen::Matrix4f transformTR2TC;

	bool FR_hascome;
	bool FL_hascome;
	bool TL_hascome;
	bool TC_hascome;
	bool TR_hascome;



};

}

#include "multi_lidar_calib_class.cpp"
#endif
