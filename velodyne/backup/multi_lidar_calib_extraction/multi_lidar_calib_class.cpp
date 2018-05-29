#include "multi_lidar_calib.h"

namespace MSY_Denso{

msy_multi_lidar::msy_multi_lidar()
:   CloudFR_sel(new pcl::PointCloud<pcl::PointXYZI>()),
    CloudFL_sel(new pcl::PointCloud<pcl::PointXYZI>()),
    CloudTL_sel(new pcl::PointCloud<pcl::PointXYZI>()),
    CloudTC_sel(new pcl::PointCloud<pcl::PointXYZI>()),
    CloudTR_sel(new pcl::PointCloud<pcl::PointXYZI>()),
        _regionLabel(),
        _scanNeighborPicked(),
        _regionSortIndices(),
        _regionCurvature(),
        _laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>()),
        _coeffSel(new pcl::PointCloud<pcl::PointXYZI>())

{
    ;
}

void msy_multi_lidar::lidar_read1( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg){

    FR_hascome = true;
    _timeFR = laserCloudMsg->header.stamp;

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*laserCloudMsg, CloudFR);

    // sensor_msgs::PointCloud2 msy_laserCloud;
    // pcl::toROSMsg(*CloudFR_sel, msy_laserCloud);
    // msy_laserCloud.header.stamp = _timeFR;
    // msy_laserCloud.header.frame_id = "velodyne";
    // pub1.publish(msy_laserCloud);

}
void msy_multi_lidar::lidar_read2( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg){

    FL_hascome = true;
    _timeFL = laserCloudMsg->header.stamp;

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*laserCloudMsg, CloudFL);
}
void msy_multi_lidar::lidar_read3( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg){

    TL_hascome = true;
    _timeTL = laserCloudMsg->header.stamp;

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*laserCloudMsg, CloudTL);
}
void msy_multi_lidar::lidar_read4( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg){

    TC_hascome = true;
    _timeTC = laserCloudMsg->header.stamp;

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*laserCloudMsg, CloudTC);
    pcl::fromROSMsg(*laserCloudMsg, CloudTC_ni);

    //     sensor_msgs::PointCloud2 msy_laserCloud;
    // pcl::toROSMsg(*CloudTC_sel, msy_laserCloud);
    // msy_laserCloud.header.stamp = _timeTC;
    // msy_laserCloud.header.frame_id = "velodyne";
    // pub4.publish(msy_laserCloud);

}
void msy_multi_lidar::lidar_read5( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg){

    TR_hascome = true;
    _timeTR = laserCloudMsg->header.stamp;

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*laserCloudMsg, CloudTR);
    pcl::fromROSMsg(*laserCloudMsg, CloudTR_ni);

    //         CloudTR_sel->clear();
    // size_t size_TR = CloudTR.points.size();
    // for (size_t i=0;i<size_TR; i++)
    // {

    //     if (CloudTR[i].z<0)
    //     {
    //         // std::cout<<CloudTC[i].z<<std::endl;
    //         CloudTR_sel->push_back(CloudTR[i]);
    //     }
    //         // CloudFR_sel.push_back(CloudFR[i]);
    // }

    //     sensor_msgs::PointCloud2 msy_laserCloud;
    // pcl::toROSMsg(*CloudTR_sel, msy_laserCloud);
    // msy_laserCloud.header.stamp = _timeTR;
    // msy_laserCloud.header.frame_id = "velodyne";
    // pub5.publish(msy_laserCloud);

}

void msy_multi_lidar::setup(ros::NodeHandle& nh){

ringNum = 32;
lowerBound = -30.67f;
upperBound = 10.67f;
nScanRings = 32;

scanPeriod = 0.1;
imuHistorySize = 200;
nFeatureRegions = 6;
curvatureRegion = 5;
maxCornerSharp = 2;
maxCornerLessSharp = 10 * maxCornerSharp;
maxSurfaceFlat = 4;
lessFlatFilterSize = 0.2;
surfaceCurvatureThreshold = 0.1;



FR_hascome = false;
FL_hascome = false;
TL_hascome = false;
TC_hascome = false;
TR_hascome = false;

sub1 = nh.subscribe("ns1/velodyne_points", 10, &msy_multi_lidar::lidar_read1, this);
pub1 = nh.advertise<sensor_msgs::PointCloud2>("/transferred_FR", 10);
sub2 = nh.subscribe("ns2/velodyne_points", 10, &msy_multi_lidar::lidar_read2, this);
pub2 = nh.advertise<sensor_msgs::PointCloud2>("/transferred_FL", 10);
sub3 = nh.subscribe("ns3/velodyne_points", 10, &msy_multi_lidar::lidar_read3, this);
pub3 = nh.advertise<sensor_msgs::PointCloud2>("/transferred_TL", 10);
sub4 = nh.subscribe("ns4/velodyne_points", 10, &msy_multi_lidar::lidar_read4, this);
pub4 = nh.advertise<sensor_msgs::PointCloud2>("/transferred_TC", 10);
sub5 = nh.subscribe("ns5/velodyne_points", 10, &msy_multi_lidar::lidar_read5, this);
pub5 = nh.advertise<sensor_msgs::PointCloud2>("/transferred_TR", 10);

}

void msy_multi_lidar::spin(){
    ros::Rate rate(10);
    // ros::spinOnce();
    rate.sleep();
    // loop until shutdown
    while (process() && ros::ok()) {
    ros::spinOnce();
    rate.sleep();
    }
}

void msy_multi_lidar::transformTowardTarget(const pcl::PointXYZI& pi, pcl::PointXYZI& po)
{


  po.x = pi.x;
  po.y = pi.y;
  po.z = pi.z;

  Angle rx = - transform_rx;
  Angle ry = - transform_ry;
  Angle rz = - transform_rz;
  rotateZXY(po, rz, rx, ry);

  po.x = po.x - transform_tx;
  po.y = po.y - transform_ty;
  po.z = po.z - transform_tz;
  po.intensity = pi.intensity;

}


void msy_multi_lidar::FinalTransform(pcl::PointCloud<pcl::PointXYZI>& cloud)
{
  size_t cloudSize = cloud.points.size();

  for (size_t i = 0; i < cloudSize; i++) {

    pcl::PointXYZI& point = cloud.points[i];

    Angle rx = - transform_rx;
    Angle ry = - transform_ry;
    Angle rz = - transform_rz;
    rotateZXY(point, rz, rx, ry);

    point.x = point.x - transform_tx;
    point.y = point.y - transform_ty;
    point.z = point.z - transform_tz;


  }

}

void msy_multi_lidar::LM( pcl::PointCloud<pcl::PointXYZI> & Target_corner,
       pcl::PointCloud<pcl::PointXYZI> & Target_surf,
       pcl::PointCloud<pcl::PointXYZI> & Source_corner,
       pcl::PointCloud<pcl::PointXYZI> & Source_surf)
{

    ROS_INFO("                      LM");

  size_t _maxIterations = 200;

  transform_rx = 0.0;
  transform_ry = 0.0;
  transform_rz = 0.0;
  transform_tx = 0.0;
  transform_ty = 0.0;
  transform_tz = 0.0;

  std::vector<int> _pointSearchCornerInd1;    ///< first corner point search index buffer
  std::vector<int> _pointSearchCornerInd2;    ///< second corner point search index buffer

  std::vector<int> _pointSearchSurfInd1;    ///< first surface point search index buffer
  std::vector<int> _pointSearchSurfInd2;    ///< second surface point search index buffer
  std::vector<int> _pointSearchSurfInd3;    ///< third surface point search index buffer

  nanoflann::KdTreeFLANN<pcl::PointXYZI> _lastCornerKDTree;   ///< last corner cloud KD-tree
  nanoflann::KdTreeFLANN<pcl::PointXYZI> _lastSurfaceKDTree;  ///< last surface cloud KD-tree

    pcl::PointCloud<pcl::PointXYZI>::Ptr _lastCornerCloud(new pcl::PointCloud<pcl::PointXYZI>());    ///< last corner points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _lastSurfaceCloud(new pcl::PointCloud<pcl::PointXYZI>());   ///< last surface points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>());    ///< last corner points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>());   ///< last surface points cloud

    *_lastCornerCloud = Target_corner;
    *_lastSurfaceCloud = Target_surf;
    *_cornerPointsSharp = Source_corner;
    *_surfPointsFlat = Source_surf;

    _lastCornerKDTree.setInputCloud(_lastCornerCloud);
    _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);


  pcl::PointXYZI coeff;
  bool isDegenerate = false;
  Eigen::Matrix<float,6,6> matP;

  size_t lastCornerCloudSize = _lastCornerCloud->points.size();
  size_t lastSurfaceCloudSize = _lastSurfaceCloud->points.size();

  // for (size_t ii = 0; ii < lastSurfaceCloudSize; ii++)
  // {
  //   std::cout<<"intensity:   " << _lastSurfaceCloud->points[ii].intensity << std::endl;
  // }


  if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100) {
    std::vector<int> pointSearchInd(1);
    std::vector<float> pointSearchSqDis(1);
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(*_cornerPointsSharp, *_cornerPointsSharp, indices);
    size_t cornerPointsSharpNum = _cornerPointsSharp->points.size();
    size_t surfPointsFlatNum = _surfPointsFlat->points.size();

    std::cout<<"cornerPointsSharpNum:"<<cornerPointsSharpNum<<"   surfPointsFlatNum:"<<surfPointsFlatNum<<std::endl;

    _pointSearchCornerInd1.resize(cornerPointsSharpNum);
    _pointSearchCornerInd2.resize(cornerPointsSharpNum);
    _pointSearchSurfInd1.resize(surfPointsFlatNum);
    _pointSearchSurfInd2.resize(surfPointsFlatNum);
    _pointSearchSurfInd3.resize(surfPointsFlatNum);


    for (size_t iterCount = 0; iterCount < _maxIterations; iterCount++) {
      pcl::PointXYZI pointSel, pointProj, tripod1, tripod2, tripod3;
      _laserCloudOri->clear();
      _coeffSel->clear();

      for (int i = 0; i < cornerPointsSharpNum; i++) {
        transformTowardTarget(_cornerPointsSharp->points[i], pointSel);
        // pointSel = _cornerPointsSharp->points[i];

        if (iterCount % 1 == 0) {
          pcl::removeNaNFromPointCloud(*_lastCornerCloud, *_lastCornerCloud, indices);
          _lastCornerKDTree.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);


          int closestPointInd = -1, minPointInd2 = -1;
          if (pointSearchSqDis[0] < 25) {
            closestPointInd = pointSearchInd[0];


            // if (i == 20)
            // std::cout << "corner pointSearchInd for 20th point: " << closestPointInd << std::endl;



            int closestPointScan = int(_lastCornerCloud->points[closestPointInd].intensity);

            float pointSqDis, minPointSqDis2 = 25;
            for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
              if (int(_lastCornerCloud->points[j].intensity) > closestPointScan + 2.5) {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

              if (int(_lastCornerCloud->points[j].intensity) > closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              }
            }
            for (int j = closestPointInd - 1; j >= 0; j--) {
              if (int(_lastCornerCloud->points[j].intensity) < closestPointScan - 2.5) {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

              if (int(_lastCornerCloud->points[j].intensity) < closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              }
            }
          }

          _pointSearchCornerInd1[i] = closestPointInd;
          _pointSearchCornerInd2[i] = minPointInd2;
        }

        if (_pointSearchCornerInd2[i] >= 0) {
          tripod1 = _lastCornerCloud->points[_pointSearchCornerInd1[i]];
          tripod2 = _lastCornerCloud->points[_pointSearchCornerInd2[i]];

          float x0 = pointSel.x;
          float y0 = pointSel.y;
          float z0 = pointSel.z;
          float x1 = tripod1.x;
          float y1 = tripod1.y;
          float z1 = tripod1.z;
          float x2 = tripod2.x;
          float y2 = tripod2.y;
          float z2 = tripod2.z;

          float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                            * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                            + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                              * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                            + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                              * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

          float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

          float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                      + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

          float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                       - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

          float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                       + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

          float ld2 = a012 / l12;

          // TODO: Why writing to a variable that's never read?
          pointProj = pointSel;
          pointProj.x -= la * ld2;
          pointProj.y -= lb * ld2;
          pointProj.z -= lc * ld2;

          float s = 1;
          if (iterCount >= 5) {
            s = 1 - 1.8f * fabs(ld2);
          }

          coeff.x = s * la;
          coeff.y = s * lb;
          coeff.z = s * lc;
          coeff.intensity = s * ld2;

          // if (iterCount % 5 != 0)
          // {
          //   std::cout << "coeff.x: "<<coeff.x<<std::endl;
          // }

          if (s > 0.1 && ld2 != 0) {
            _laserCloudOri->push_back(_cornerPointsSharp->points[i]);
            _coeffSel->push_back(coeff);
          }
        }
      }

      for (int i = 0; i < surfPointsFlatNum; i++) {
        transformTowardTarget(_surfPointsFlat->points[i], pointSel);
        // pointSel = _surfPointsFlat->points[i];

        if (iterCount % 1 == 0) {
          _lastSurfaceKDTree.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
          int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
          if (pointSearchSqDis[0] < 25) {
            closestPointInd = pointSearchInd[0];


            // if (i == 20)
            // std::cout << "corner pointSearchInd for 20th point: " << closestPointInd << std::endl;


            int closestPointScan = int(_lastSurfaceCloud->points[closestPointInd].intensity);

            float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
            for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
              if (int(_lastSurfaceCloud->points[j].intensity) > closestPointScan + 2.5) {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

              if (int(_lastSurfaceCloud->points[j].intensity) <= closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              } else {
                if (pointSqDis < minPointSqDis3) {
                  minPointSqDis3 = pointSqDis;
                  minPointInd3 = j;
                }
              }
            }
            for (int j = closestPointInd - 1; j >= 0; j--) {
              if (int(_lastSurfaceCloud->points[j].intensity) < closestPointScan - 2.5) {
                break;
              }

              pointSqDis = calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

              if (int(_lastSurfaceCloud->points[j].intensity) >= closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              } else {
                if (pointSqDis < minPointSqDis3) {
                  minPointSqDis3 = pointSqDis;
                  minPointInd3 = j;
                }
              }
            }
          }

          _pointSearchSurfInd1[i] = closestPointInd;
          _pointSearchSurfInd2[i] = minPointInd2;
          _pointSearchSurfInd3[i] = minPointInd3;
        }

        if (_pointSearchSurfInd2[i] >= 0 && _pointSearchSurfInd3[i] >= 0) {
          tripod1 = _lastSurfaceCloud->points[_pointSearchSurfInd1[i]];
          tripod2 = _lastSurfaceCloud->points[_pointSearchSurfInd2[i]];
          tripod3 = _lastSurfaceCloud->points[_pointSearchSurfInd3[i]];

          float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
                     - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
          float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
                     - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
          float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
                     - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
          float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

          float ps = sqrt(pa * pa + pb * pb + pc * pc);
          pa /= ps;
          pb /= ps;
          pc /= ps;
          pd /= ps;

          float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

          // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
          pointProj = pointSel;
          pointProj.x -= pa * pd2;
          pointProj.y -= pb * pd2;
          pointProj.z -= pc * pd2;

          float s = 1;
          if (iterCount >= 5) {
            s = 1 - 1.8f * fabs(pd2) / sqrt(calcPointDistance(pointSel));
          }

          coeff.x = s * pa;
          coeff.y = s * pb;
          coeff.z = s * pc;
          coeff.intensity = s * pd2;

          if (s > 0.1 && pd2 != 0) {
            _laserCloudOri->push_back(_surfPointsFlat->points[i]);
            _coeffSel->push_back(coeff);
          }
        }
      }

      int pointSelNum = _laserCloudOri->points.size();
      if (pointSelNum < 10) {
        continue;
      }

      Eigen::Matrix<float,Eigen::Dynamic,6> matA(pointSelNum, 6);
      Eigen::Matrix<float,6,Eigen::Dynamic> matAt(6,pointSelNum);
      Eigen::Matrix<float,6,6> matAtA;
      Eigen::VectorXf matB(pointSelNum);
      Eigen::Matrix<float,6,1> matAtB;
      Eigen::Matrix<float,6,1> matX;

      for (int i = 0; i < pointSelNum; i++) {
        const pcl::PointXYZI& pointOri = _laserCloudOri->points[i];
        coeff = _coeffSel->points[i];

        float s = 1;

        // float srx = sin(s * _transform.rot_x.rad());
        // float crx = cos(s * _transform.rot_x.rad());
        // float sry = sin(s * _transform.rot_y.rad());
        // float cry = cos(s * _transform.rot_y.rad());
        // float srz = sin(s * _transform.rot_z.rad());
        // float crz = cos(s * _transform.rot_z.rad());
        // float tx = s * _transform.pos.x();
        // float ty = s * _transform.pos.y();
        // float tz = s * _transform.pos.z();

        float srx = sin(s * transform_rx);
        float crx = cos(s * transform_rx);
        float sry = sin(s * transform_ry);
        float cry = cos(s * transform_ry);
        float srz = sin(s * transform_rz);
        float crz = cos(s * transform_rz);
        float tx = s * transform_tx;
        float ty = s * transform_ty;
        float tz = s * transform_tz;


        float arx = (-s*crx*sry*srz*pointOri.x + s*crx*crz*sry*pointOri.y + s*srx*sry*pointOri.z
                     + s*tx*crx*sry*srz - s*ty*crx*crz*sry - s*tz*srx*sry) * coeff.x
                    + (s*srx*srz*pointOri.x - s*crz*srx*pointOri.y + s*crx*pointOri.z
                       + s*ty*crz*srx - s*tz*crx - s*tx*srx*srz) * coeff.y
                    + (s*crx*cry*srz*pointOri.x - s*crx*cry*crz*pointOri.y - s*cry*srx*pointOri.z
                       + s*tz*cry*srx + s*ty*crx*cry*crz - s*tx*crx*cry*srz) * coeff.z;

        float ary = ((-s*crz*sry - s*cry*srx*srz)*pointOri.x
                     + (s*cry*crz*srx - s*sry*srz)*pointOri.y - s*crx*cry*pointOri.z
                     + tx*(s*crz*sry + s*cry*srx*srz) + ty*(s*sry*srz - s*cry*crz*srx)
                     + s*tz*crx*cry) * coeff.x
                    + ((s*cry*crz - s*srx*sry*srz)*pointOri.x
                       + (s*cry*srz + s*crz*srx*sry)*pointOri.y - s*crx*sry*pointOri.z
                       + s*tz*crx*sry - ty*(s*cry*srz + s*crz*srx*sry)
                       - tx*(s*cry*crz - s*srx*sry*srz)) * coeff.z;

        float arz = ((-s*cry*srz - s*crz*srx*sry)*pointOri.x + (s*cry*crz - s*srx*sry*srz)*pointOri.y
                     + tx*(s*cry*srz + s*crz*srx*sry) - ty*(s*cry*crz - s*srx*sry*srz)) * coeff.x
                    + (-s*crx*crz*pointOri.x - s*crx*srz*pointOri.y
                       + s*ty*crx*srz + s*tx*crx*crz) * coeff.y
                    + ((s*cry*crz*srx - s*sry*srz)*pointOri.x + (s*crz*sry + s*cry*srx*srz)*pointOri.y
                       + tx*(s*sry*srz - s*cry*crz*srx) - ty*(s*crz*sry + s*cry*srx*srz)) * coeff.z;

        float atx = -s*(cry*crz - srx*sry*srz) * coeff.x + s*crx*srz * coeff.y
                    - s*(crz*sry + cry*srx*srz) * coeff.z;

        float aty = -s*(cry*srz + crz*srx*sry) * coeff.x - s*crx*crz * coeff.y
                    - s*(sry*srz - cry*crz*srx) * coeff.z;

        float atz = s*crx*sry * coeff.x - s*srx * coeff.y - s*crx*cry * coeff.z;

        float d2 = coeff.intensity;

        matA(i, 0) = arx;
        matA(i, 1) = ary;
        matA(i, 2) = arz;
        matA(i, 3) = atx;
        matA(i, 4) = aty;
        matA(i, 5) = atz;
        matB(i, 0) = -0.05 * d2;
      }
      matAt = matA.transpose();
      matAtA = matAt * matA;
      matAtB = matAt * matB;

      matX = matAtA.colPivHouseholderQr().solve(matAtB);

      if (iterCount == 0) {
        Eigen::Matrix<float,1,6> matE;
        Eigen::Matrix<float,6,6> matV;
        Eigen::Matrix<float,6,6> matV2;

        Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,6, 6> > esolver(matAtA);
        matE = esolver.eigenvalues().real();
        matV = esolver.eigenvectors().real();

        matV2 = matV;

        isDegenerate = false;
        float eignThre[6] = {10, 10, 10, 10, 10, 10};
        for (int i = 0; i < 6; i++) {
          if (matE(0, i) < eignThre[i]) {
            for (int j = 0; j < 6; j++) {
              matV2(i, j) = 0;
            }
            isDegenerate = true;
          } else {
            break;
          }
        }
        matP = matV.inverse() * matV2;
      }

      if (isDegenerate) {
        Eigen::Matrix<float,6,1> matX2;
        matX2 = matX;
        matX = matP * matX2;
      }

      // _transform.rot_x = _transform.rot_x.rad() + matX(0, 0);
      // _transform.rot_y = _transform.rot_y.rad() + matX(1, 0);
      // _transform.rot_z = _transform.rot_z.rad() + matX(2, 0);
      // _transform.pos.x() += matX(3, 0);
      // _transform.pos.y() += matX(4, 0);
      // _transform.pos.z() += matX(5, 0);

      transform_rx += matX(0, 0);
      transform_ry += matX(1, 0);
      transform_rz += matX(2, 0);
      transform_tx += matX(3, 0);
      transform_ty += matX(4, 0);
      transform_tz += matX(5, 0);

      // if( !pcl_isfinite(_transform.rot_x.rad()) ) _transform.rot_x = Angle();
      // if( !pcl_isfinite(_transform.rot_y.rad()) ) _transform.rot_y = Angle();
      // if( !pcl_isfinite(_transform.rot_z.rad()) ) _transform.rot_z = Angle();

      // if( !pcl_isfinite(_transform.pos.x()) ) _transform.pos.x() = 0.0;
      // if( !pcl_isfinite(_transform.pos.y()) ) _transform.pos.y() = 0.0;
      // if( !pcl_isfinite(_transform.pos.z()) ) _transform.pos.z() = 0.0;


      if( !pcl_isfinite(transform_rx) ) transform_rx = 0.0;
      if( !pcl_isfinite(transform_ry) ) transform_ry = 0.0;
      if( !pcl_isfinite(transform_rz) ) transform_rz = 0.0;
      if( !pcl_isfinite(transform_tx) ) transform_tx = 0.0;
      if( !pcl_isfinite(transform_ty) ) transform_ty = 0.0;
      if( !pcl_isfinite(transform_tz) ) transform_tz = 0.0;


      float deltaR = sqrt(pow(rad2deg(matX(0, 0)), 2) +
                          pow(rad2deg(matX(1, 0)), 2) +
                          pow(rad2deg(matX(2, 0)), 2));
      float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                          pow(matX(4, 0) * 100, 2) +
                          pow(matX(5, 0) * 100, 2));

      float _deltaTAbort = 0.1;     ///< optimization abort threshold for deltaT
      float _deltaRAbort = 0.1;     ///< optimization abort threshold for deltaR


      if (deltaR < _deltaRAbort && deltaT < _deltaTAbort) {
        break;
      }
    }
  }

  std::cout<<"_transform.rot_x.rad():   " << transform_rx << std::endl;
  std::cout<<"_transform.rot_y.rad():   " << transform_ry << std::endl;
  std::cout<<"_transform.rot_z.rad():   " << transform_rz << std::endl;

  std::cout<<"_transform.pos.x():   " << transform_tx << std::endl;
  std::cout<<"_transform.pos.y():   " << transform_ty << std::endl;
  std::cout<<"_transform.pos.z():   " << transform_tz << std::endl;

  sensor_msgs::PointCloud2 msy_laserCloudqqqq;
  pcl::toROSMsg(*_surfPointsFlat, msy_laserCloudqqqq);
  msy_laserCloudqqqq.header.stamp = _timeTR;
  msy_laserCloudqqqq.header.frame_id = "velodyne";
  pub1.publish(msy_laserCloudqqqq);

  // FinalTransform(_cornerPointsSharp);
  // FinalTransform(_surfPointsFlat);
  FinalTransform(CloudTR);

  // std::cout<<"_transform.rot_x.rad():   " << matX(0, 0) << std::endl;
  // std::cout<<"_transform.rot_y.rad():   " << matX(1, 0) << std::endl;
  // std::cout<<"_transform.rot_z.rad():   " << matX(2, 0) << std::endl;

  // std::cout<<"_transform.pos.x():   " << matX(3, 0) << std::endl;
  // std::cout<<"_transform.pos.y():   " << matX(4, 0) << std::endl;
  // std::cout<<"_transform.pos.z():   " << matX(5, 0) << std::endl;




  pcl::toROSMsg(*_surfPointsFlat, msy_laserCloudqqqq);
  msy_laserCloudqqqq.header.stamp = _timeTR;
  msy_laserCloudqqqq.header.frame_id = "velodyne";
  pub2.publish(msy_laserCloudqqqq);

  pcl::toROSMsg(CloudTR, msy_laserCloudqqqq);
  msy_laserCloudqqqq.header.stamp = _timeTR;
  msy_laserCloudqqqq.header.frame_id = "velodyne";
  pub3.publish(msy_laserCloudqqqq);


  pcl::toROSMsg(CloudTC, msy_laserCloudqqqq);
  msy_laserCloudqqqq.header.stamp = _timeTC;
  msy_laserCloudqqqq.header.frame_id = "velodyne";
  pub4.publish(msy_laserCloudqqqq);



}

bool msy_multi_lidar::process()
{
  pcl::PointCloud<pcl::PointXYZI> PC_Target_corner;
  pcl::PointCloud<pcl::PointXYZI> PC_Target_surf;

  pcl::PointCloud<pcl::PointXYZI> PC_Source_corner;
  pcl::PointCloud<pcl::PointXYZI> PC_Source_surf;

  Registration(CloudTC_ni, _timeTC, PC_Target_corner, PC_Target_surf);
  Registration(CloudTR_ni, _timeTR, PC_Source_corner, PC_Source_surf);


  LM(PC_Target_corner, PC_Target_surf, PC_Source_corner, PC_Source_surf);




  // if (TC_hascome ) return false;
  // else return true; 
  return true; 
}

int msy_multi_lidar::getRingForAngle(const float& angle) {

  factor = (nScanRings - 1) / (upperBound - lowerBound);
  return int(((angle * 180 / M_PI) - lowerBound) * factor + 0.5);
}

void msy_multi_lidar::setScanBuffersFor(const size_t& startIdx,
                                         const size_t& endIdx, 
                                         pcl::PointCloud<pcl::PointXYZI> & _laserCloud)
{
  // resize buffers
  size_t scanSize = endIdx - startIdx + 1;
  _scanNeighborPicked.assign(scanSize, 0);

  // mark unreliable points as picked
  for (size_t i = startIdx + curvatureRegion; i < endIdx - curvatureRegion; i++) {
    const pcl::PointXYZI& previousPoint = (_laserCloud[i - 1]);
    const pcl::PointXYZI& point = (_laserCloud[i]);
    const pcl::PointXYZI& nextPoint = (_laserCloud[i + 1]);

    float diffNext = calcSquaredDiff(nextPoint, point);

    if (diffNext > 0.1) {
      float depth1 = calcPointDistance(point);
      float depth2 = calcPointDistance(nextPoint);

      if (depth1 > depth2) {
        float weighted_distance = std::sqrt(calcSquaredDiff(nextPoint, point, depth2 / depth1)) / depth2;

        if (weighted_distance < 0.1) {
          std::fill_n(&_scanNeighborPicked[i - startIdx - curvatureRegion], curvatureRegion + 1, 1);

          continue;
        }
      } else {
        float weighted_distance = std::sqrt(calcSquaredDiff(point, nextPoint, depth1 / depth2)) / depth1;

        if (weighted_distance < 0.1) {
          std::fill_n(&_scanNeighborPicked[i - startIdx + 1], curvatureRegion + 1, 1);
        }
      }
    }

    float diffPrevious = calcSquaredDiff(point, previousPoint);
    float dis = calcSquaredPointDistance(point);

    if (diffNext > 0.0002 * dis && diffPrevious > 0.0002 * dis) {
      _scanNeighborPicked[i - startIdx] = 1;
    }
  }
}


void msy_multi_lidar::setRegionBuffersFor(const size_t& startIdx,
                                           const size_t& endIdx, 
                                           pcl::PointCloud<pcl::PointXYZI> & _laserCloud)
{
  // resize buffers
  size_t regionSize = endIdx - startIdx + 1;
  _regionCurvature.resize(regionSize);
  _regionSortIndices.resize(regionSize);
  _regionLabel.assign(regionSize, SURFACE_LESS_FLAT);

  // calculate point curvatures and reset sort indices
  float pointWeight = -2 * curvatureRegion;

  for (size_t i = startIdx, regionIdx = 0; i <= endIdx; i++, regionIdx++) {
    float diffX = pointWeight * _laserCloud[i].x;
    float diffY = pointWeight * _laserCloud[i].y;
    float diffZ = pointWeight * _laserCloud[i].z;

    for (int j = 1; j <= curvatureRegion; j++) {
      diffX += _laserCloud[i + j].x + _laserCloud[i - j].x;
      diffY += _laserCloud[i + j].y + _laserCloud[i - j].y;
      diffZ += _laserCloud[i + j].z + _laserCloud[i - j].z;
    }

    _regionCurvature[regionIdx] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    _regionSortIndices[regionIdx] = i;
  }

  // sort point curvatures
  for (size_t i = 1; i < regionSize; i++) {
    for (size_t j = i; j >= 1; j--) {
      if (_regionCurvature[_regionSortIndices[j] - startIdx] < _regionCurvature[_regionSortIndices[j - 1] - startIdx]) {
        std::swap(_regionSortIndices[j], _regionSortIndices[j - 1]);
      }
    }
  }
}



void msy_multi_lidar::markAsPicked(const size_t& cloudIdx,
                                    const size_t& scanIdx, 
                                    pcl::PointCloud<pcl::PointXYZI> & _laserCloud)
{
  _scanNeighborPicked[scanIdx] = 1;

  for (int i = 1; i <= curvatureRegion; i++) {
    if (calcSquaredDiff(_laserCloud[cloudIdx + i], _laserCloud[cloudIdx + i - 1]) > 0.05) {
      break;
    }

    _scanNeighborPicked[scanIdx + i] = 1;
  }

  for (int i = 1; i <= curvatureRegion; i++) {
    if (calcSquaredDiff(_laserCloud[cloudIdx - i], _laserCloud[cloudIdx - i + 1]) > 0.05) {
      break;
    }

    _scanNeighborPicked[scanIdx - i] = 1;
  }
}

void msy_multi_lidar::Registration(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn,
                                    const ros::Time& scanTime,
                                    pcl::PointCloud<pcl::PointXYZI> & PC_feature_corner,
                                    pcl::PointCloud<pcl::PointXYZI> & PC_feature_surf)
{
    ROS_INFO("                      Registration");
  size_t cloudSize = laserCloudIn.size();
  std::cout <<cloudSize<< std::endl;

  if (cloudSize == 0) return;

  float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);

  float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y,
                             laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);

  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }


  bool halfPassed = false;
  pcl::PointXYZI point;
  std::vector<pcl::PointCloud<pcl::PointXYZI> > laserCloudScans(ringNum);

  // extract valid points from input cloud
  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn[i].x;
    point.y = laserCloudIn[i].y;
    point.z = laserCloudIn[i].z;

    // skip NaN and INF valued points
    if (!pcl_isfinite(point.x) ||
        !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    // skip zero valued points
    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
      continue;
    }

    // calculate vertical point angle and scan ID
    float angle = std::atan(point.z / std::sqrt(point.x * point.x + point.y * point.y));
    int scanID = getRingForAngle(angle);
    if (scanID >= ringNum || scanID < 0 ){
      continue;
    }

    // calculate horizontal point angle
    float ori = -std::atan2(point.y, point.x);
    if (!halfPassed) {
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * M_PI;

      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    point.intensity = scanID;

    laserCloudScans[scanID].push_back(point);

  
  }


  pcl::PointCloud<pcl::PointXYZI> _laserCloud;
  std::vector< std::pair<size_t, size_t> > _scanIndices;
    _scanIndices.clear();


  cloudSize = 0;
  for (int i = 0; i < ringNum ; i++) { 
    _laserCloud += laserCloudScans[i];

    std::pair<size_t, size_t> range(cloudSize, 0);
    cloudSize += laserCloudScans[i].size();
    range.second = cloudSize > 0 ? cloudSize - 1 : 0;
    _scanIndices.push_back(range);
  }

    // sensor_msgs::PointCloud2 msy_laserCloudqqqq;
    // pcl::toROSMsg(_laserCloud, msy_laserCloudqqqq);
    // msy_laserCloudqqqq.header.stamp = _timeTC;
    // msy_laserCloudqqqq.header.frame_id = "velodyne";
    // pub3.publish(msy_laserCloudqqqq);

/**************************************************************/
/***************************Extraction*************************/
/**************************************************************/
  pcl::PointCloud<pcl::PointXYZI> _cornerPointsSharp;      ///< sharp corner points cloud
  pcl::PointCloud<pcl::PointXYZI> _cornerPointsLessSharp;  ///< less sharp corner points cloud
  pcl::PointCloud<pcl::PointXYZI> _surfacePointsFlat;      ///< flat surface points cloud
  pcl::PointCloud<pcl::PointXYZI> _surfacePointsLessFlat;  ///< less flat surface points cloud
    _laserCloud.clear();
    _cornerPointsSharp.clear();
    _cornerPointsLessSharp.clear();
    _surfacePointsFlat.clear();
    _surfacePointsLessFlat.clear();



  uint16_t beginIdx = 0;
  size_t nScans = _scanIndices.size();

    for (size_t i = beginIdx; i < nScans; i++) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>);
    size_t scanStartIdx = _scanIndices[i].first;
    size_t scanEndIdx = _scanIndices[i].second;

    // skip empty scans
    if (scanEndIdx <= scanStartIdx + 2 * curvatureRegion) {
      continue;
    }

    setScanBuffersFor(scanStartIdx, scanEndIdx, _laserCloud);

    // extract features from equally sized scan regions
    for (int j = 0; j < nFeatureRegions; j++) {
      size_t sp = ((scanStartIdx + curvatureRegion) * (nFeatureRegions - j)
                   + (scanEndIdx - curvatureRegion) * j) / nFeatureRegions;
      size_t ep = ((scanStartIdx + curvatureRegion) * (nFeatureRegions - 1 - j)
                   + (scanEndIdx - curvatureRegion) * (j + 1)) / nFeatureRegions - 1;

      // skip empty regions
      if (ep <= sp) {
        continue;
      }

      size_t regionSize = ep - sp + 1;

      // reset region buffers
      setRegionBuffersFor(sp, ep, _laserCloud);

      // extract corner features
      int largestPickedNum = 0;
      for (size_t k = regionSize; k > 0 && largestPickedNum < maxCornerLessSharp;) {
        size_t idx = _regionSortIndices[--k];
        size_t scanIdx = idx - scanStartIdx;
        size_t regionIdx = idx - sp;

        if (_scanNeighborPicked[scanIdx] == 0 &&
            _regionCurvature[regionIdx] > surfaceCurvatureThreshold) {

          largestPickedNum++;
          if (largestPickedNum <= maxCornerSharp) {
            _regionLabel[regionIdx] = CORNER_SHARP;
            _cornerPointsSharp.push_back(_laserCloud[idx]);
          } else {
            _regionLabel[regionIdx] = CORNER_LESS_SHARP;
          }
          _cornerPointsLessSharp.push_back(_laserCloud[idx]);

          markAsPicked(idx, scanIdx, _laserCloud);
        }
      }

      // extract flat surface features
      int smallestPickedNum = 0;
      for (int k = 0; k < regionSize && smallestPickedNum < maxSurfaceFlat; k++) {
        size_t idx = _regionSortIndices[k];
        size_t scanIdx = idx - scanStartIdx;
        size_t regionIdx = idx - sp;

        if (_scanNeighborPicked[scanIdx] == 0 &&
            _regionCurvature[regionIdx] < surfaceCurvatureThreshold) {

          smallestPickedNum++;
          _regionLabel[regionIdx] = SURFACE_FLAT;
          _surfacePointsFlat.push_back(_laserCloud[idx]);

          markAsPicked(idx, scanIdx, _laserCloud);
        }
      }

      // extract less flat surface features
      for (int k = 0; k < regionSize; k++) {
        if (_regionLabel[k] <= SURFACE_LESS_FLAT) {
          surfPointsLessFlatScan->push_back(_laserCloud[sp + k]);
        }
      }
    }

    // down size less flat surface point cloud of current scan
    pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(lessFlatFilterSize, lessFlatFilterSize, lessFlatFilterSize);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    _surfacePointsLessFlat += surfPointsLessFlatScanDS;


    PC_feature_corner.clear();
    PC_feature_corner = _cornerPointsSharp;
    PC_feature_surf.clear();
    PC_feature_surf = _surfacePointsFlat;



  // std::cout <<"_cornerPointsSharp.size: "<<_cornerPointsSharp.size()<< std::endl;
  // std::cout <<"_surfacePointsFlat.size: "<<_surfacePointsFlat.size()<< std::endl;

    // sensor_msgs::PointCloud2 msy_laserCloud;
    // pcl::toROSMsg(PC_feature_corner, msy_laserCloud);
    // msy_laserCloud.header.stamp = _timeTC;
    // msy_laserCloud.header.frame_id = "velodyne";
    // pub1.publish(msy_laserCloud);

    // pcl::toROSMsg(PC_feature_surf, msy_laserCloud);
    // msy_laserCloud.header.stamp = _timeTC;
    // msy_laserCloud.header.frame_id = "velodyne";
    // pub2.publish(msy_laserCloud);


  }

}


} //namespace end