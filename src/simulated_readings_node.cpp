#include <algorithm>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <math.h>
#include <opencv2/opencv.hpp>

using namespace std;

class SimuReadings
{
public:

  SimuReadings () : log_file_path("map.pts"), frame_id("base_link"), pixel_size(1), lineSize(20), angle_min(-M_PI_2), angle_max(M_PI_2), angle_increment(M_PI/180),
    range_max(std::numeric_limits<double>::max()), rangeStep(1)
  {
    minX = -numeric_limits<double>::max(); maxX = numeric_limits<double>::max();
    minY = -numeric_limits<double>::max(); maxY = numeric_limits<double>::max();
    minZ = -numeric_limits<double>::max(); maxZ = numeric_limits<double>::max();
    ros::NodeHandle nodeLocal("~");
    frame_id = nodeLocal.param("frame_id", frame_id);
    pixel_size = nodeLocal.param("pixel_size", pixel_size);
    lineSize = nodeLocal.param("line_size", lineSize);
    log_file_path = nodeLocal.param("log_file", log_file_path);
    minX = nodeLocal.param("minX", minX);
    maxX = nodeLocal.param("maxX", maxX);
    minY = nodeLocal.param("minY", minY);
    maxY = nodeLocal.param("maxY", maxY);
    minZ = nodeLocal.param("minZ", minZ);
    maxZ = nodeLocal.param("maxZ", maxZ);

    angle_min = nodeLocal.param("angle_min", angle_min);
    angle_max = nodeLocal.param("angle_max", angle_max);
    angle_increment = nodeLocal.param("angle_increment", angle_increment);

    range_max = nodeLocal.param("range_max", range_max);
    rangeStep = nodeLocal.param("rangeStep", rangeStep);

    std::string ns = ros::this_node::getNamespace();

    sub1 = n.subscribe("/poseStamped", 1000000, &SimuReadings::poseCallback, this);
    scan_pub = n.advertise<sensor_msgs::LaserScan>(ns+"/scan", 50);
    laser_msg.header.frame_id = frame_id;


    log_file.open(log_file_path);

    mMapImage.create((maxY - minY)/pixel_size,(maxX - minX)/pixel_size, CV_32FC1);
    loadMap();
  }

  void loadMap()
  {
    mMapImage.setTo(cv::Scalar(0));
    string line, xs, ys, zs;
    double x, y, z;

    while(getline(log_file, line))
    {
      stringstream sline(line);
      getline(sline, xs, ' ');
      getline(sline, ys, ' ');
      getline(sline, zs, ' ');
      x = stod(xs);
      y = stod(ys);
      z = stod(zs);

      if ((minX < x) && (x < maxX) &&
          (minY < y) && (y < maxY) &&
          (minZ < z) && (z < maxZ))
      {
        x = (x-minX)/pixel_size;
        y = (y-minY)/pixel_size;
        mMapImage.at<float>(mMapImage.rows-y, x) = z;
      }
    }
    cv::imshow("Robot pose", mMapImage);
    cv::waitKey(33);
  }

  void poseCallback(const geometry_msgs::PoseStamped pose_msg)
  {
    lastPose = pose_msg;
    try
    {
      listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    laser_msg.ranges.clear();

    computeLine();
//    computeScan();
    computeScanRayTracing();
    showInImage();
    laser_msg.header.stamp = pose_msg.header.stamp;
//    laser_msg.scan_time = pose_msg.header.stamp;
//    laser_msg.angle_min = minBeta;
//    laser_msg.angle_min = maxBeta;

    scan_pub.publish(laser_msg);
  }

  void computeLine()
  {
    double x,y;
    x = (transform.getOrigin().x() - minX)/pixel_size;
    y = mMapImage.rows - (transform.getOrigin().y() - minY)/pixel_size;
    ptBarco.x = std::min((double)mMapImage.cols, std::max(0.,x));
    ptBarco.y = std::min((double)mMapImage.rows, std::max(0.,y));

    double yaw = tf::getYaw(transform.getRotation());

    pt1.x = lineSize*0.5*cos(yaw + M_PI_2) - 0*sin(yaw + M_PI_2) + ptBarco.x;
    pt1.y = -(lineSize*0.5*sin(yaw + M_PI_2) - 0*cos(yaw + M_PI_2)) + ptBarco.y;

    pt2.x = lineSize*0.5*cos(yaw - M_PI_2) - 0*sin(yaw - M_PI_2) + ptBarco.x;
    pt2.y = -(lineSize*0.5*sin(yaw - M_PI_2) - 0*cos(yaw - M_PI_2)) + ptBarco.y;

    ptHeading.x = ptBarco.x + 5*cos(yaw);
    ptHeading.y = ptBarco.y - 5*sin(yaw);
  }

  void computeScan()
  {
    double c = (pt2.x - pt1.x)/lineSize;
    double s = (pt2.y - pt1.y)/lineSize;
    bool firstAngle = true;
    vector<pair<double, double>> thetaRho;
    for(double d = lineSize; d >= 0; d-=0.5)
    {
      double x = (pt1.x + d*c)*pixel_size + minX,
             y = (mMapImage.rows - (pt1.y + d*s))*pixel_size + minY,
             z = mMapImage.at<float>(std::round(pt1.y + d*s), std::round(pt1.x + d*c));

      if(z != 0)
      {
        double r = sqrt(pow(transform.getOrigin().x() - x, 2)
                      + pow(transform.getOrigin().y() - y, 2)
                      + pow(transform.getOrigin().z() - z, 2));

        double roll   = std::atan2(lineSize/2. - d, transform.getOrigin().z()-z);

        thetaRho.push_back(make_pair(roll, r));

//        if (firstAngle)
//        {
//          laser_msg.angle_min = roll;
//          firstAngle = false;
//        }
//        else
//          laser_msg.angle_max = roll;

        //if it is in the roll range...
//        laser_msg.ranges.push_back(r);
//        laser_msg.range_max = std::max(laser_msg.range_max, (float)r);
      }
    }

//    std::sort(thetaRho.begin(), thetaRho.end(), [](auto &a, auto &b) -> bool
//    {
//        return a.first < b.first;
//    });

    for(auto &it : thetaRho)
    {
        laser_msg.ranges.push_back(it.second);
        laser_msg.range_max = std::max(laser_msg.range_max, (float)it.second);
    }

    laser_msg.angle_min = thetaRho.front().first;
    laser_msg.angle_max = thetaRho.back().first;
    laser_msg.angle_increment = (laser_msg.angle_max - laser_msg.angle_min)/laser_msg.ranges.size();
  }

  void computeScanRayTracing()
  {
    double c = (pt2.x - pt1.x)/lineSize;
    double s = (pt2.y - pt1.y)/lineSize;

    for(double alfa = angle_max; alfa >= angle_min; alfa -= angle_increment)
    {
      bool hitted = false;
      for(double d = 0; d< range_max; d += rangeStep)
      {
        //Calculate the X, Y, Z of current position by RayTracing
        double dd = d*sin(alfa);
        double x = transform.getOrigin().x() + dd*c,
               y = transform.getOrigin().y() - dd*s,
               z = transform.getOrigin().z() - d*cos(alfa);

        //Calculate the X, Y for the image and retrives the value of Z
        double xI = (x-minX)/pixel_size,
               yI = mMapImage.rows - (y-minY)/pixel_size;

        if((mMapImage.cols < xI) || (xI<0))
          break;
        if((mMapImage.rows < yI) || (yI<0))
          break;

        double zI = mMapImage.at<float>(yI, xI);

        if(zI == 0)
          break;

        if(zI >= z)
        {
          double r = sqrt(pow(transform.getOrigin().x() - x, 2)
                        + pow(transform.getOrigin().y() - y, 2)
                        + pow(transform.getOrigin().z() - z, 2));

          //Validate the obstacle
          laser_msg.ranges.push_back(r);
          laser_msg.range_max = std::max(laser_msg.range_max, (float)r);
          hitted = true;
          break;
        }
      }
      if (!hitted)
        laser_msg.ranges.push_back(std::numeric_limits<double>::quiet_NaN());
    }

    laser_msg.angle_min = angle_min;
    laser_msg.angle_max = angle_max;
    laser_msg.angle_increment = angle_increment;
  }

  void showInImage()
  {
    cv::Mat temp;
    cv::cvtColor(mMapImage, temp, CV_GRAY2RGB);

    cv::circle(temp, ptBarco, 2, cv::Scalar(255,0,0), -1);
    cv::line(temp, pt1, pt2, cv::Scalar(0, 255, 0));

//    double c = (pt2.x - pt1.x)/lineSize;
//    double s = (pt2.y - pt1.y)/lineSize;
//    for(double d = 0; d <= lineSize; d+=0.5)
//    {
//      cv::circle(temp, cv::Point(pt1.x + d*c, pt1.y + d*s), 1, cv::Scalar(0,255,0), -1);
//    }
    cv::line(temp, ptBarco, ptHeading, cv::Scalar(0, 0, 255));
    cv::imshow("Robot pose", temp);
    cv::waitKey(33);
  }

private:
  ros::NodeHandle n;
  ros::Subscriber sub1;
  ros::Publisher scan_pub;
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformListener listener;
  sensor_msgs::LaserScan laser_msg;
  sensor_msgs::PointCloud mPC;
  double minX, maxX, minY, maxY, minZ, maxZ;
  double lineSize, minBeta, maxBeta;
  double pixel_size, angle_min, angle_max, angle_increment;
  double range_max, rangeStep;
  cv::Mat mMapImage;
  geometry_msgs::PoseStamped lastPose;
  std::ifstream log_file;
  cv::Point2f pt1, pt2, ptBarco, ptHeading;
  tf::StampedTransform transform;

  std::string log_file_path, frame_id;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simulated_readings_node");

  SimuReadings mSimuReadings;

  ros::spin();

}
