#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <math.h>
class GpsPose
{

public:
  double mTime, mUtmEast, mUtmNorth, mLat, mLong, mFixQuali, mNroSats, mDilut, mAlt, mGeoidH;
  std::string mLatH, mLongH, mAltU, mGeoidHU;
};

class NMEAPlayer
{
public:

  NMEAPlayer () : log_file_path("gps_converted"), frame_id("odom"), child_frame_id("base_link"), timeDiff(0), firstTime(0), EOFF(false)
  {
    start_time = ros::Time::now();
    last_time = start_time;

    ros::NodeHandle nodeLocal("~");
    frame_id = nodeLocal.param("frame_id", frame_id);
    child_frame_id = nodeLocal.param("child_frame_id", child_frame_id);
    log_file_path = nodeLocal.param("log_file", log_file_path);
    timeDiff = nodeLocal.param("time_diff", timeDiff);
    firstTime = nodeLocal.param("first_time", firstTime);

    std::string ns = ros::this_node::getNamespace();

    odom_pub = n.advertise<nav_msgs::Odometry>(ns+"/odom", 50);
    point_pub = n.advertise<geometry_msgs::PointStamped>(ns+"/pointStamped", 50);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>(ns+"/poseStamped", 50);
  }

  bool readLine(GpsPose &gpsPose)
  {
    std::string line;
    EOFF = (std::getline(log_file, line)? true : false);

    if(!EOFF)
      return EOFF;

    std::stringstream part(line);
    std::string time, utmEast, utmNorth, lat, latH, longg, longH, fixQuali, nroSats, dilut, alt, altU, geoidH, geoidHU;


    std::getline(part,time,',');
    gpsPose.mTime = std::stod(time);

    std::getline(part,utmEast,',');
    gpsPose.mUtmEast = std::stod(utmEast);

    std::getline(part,utmNorth,',');
    gpsPose.mUtmNorth = std::stod(utmNorth);

    std::getline(part,lat,',');
    gpsPose.mLat = std::stod(lat);

    std::getline(part,latH,',');
    gpsPose.mLatH = latH;

    std::getline(part,longg,',');
    gpsPose.mLong = std::stod(longg);

    std::getline(part,longH,',');
    gpsPose.mLongH = longH;

    std::getline(part,fixQuali,',');
    gpsPose.mFixQuali = std::stod(fixQuali);

    std::getline(part,nroSats,',');
    gpsPose.mNroSats = std::stod(nroSats);

    std::getline(part,dilut,',');
    gpsPose.mDilut = std::stod(dilut);

    std::getline(part,alt,',');
    gpsPose.mAlt = std::stod(alt);

    std::getline(part,altU,',');
    gpsPose.mAltU = altU;

    std::getline(part,geoidH,',');
    gpsPose.mGeoidH = std::stod(geoidH);

    std::getline(part,geoidHU,',');
    gpsPose.mGeoidHU = geoidHU;

    return EOFF;
  }

  void publishPoint()
  {
    geometry_msgs::PointStamped ptStamped;
    ptStamped.header.frame_id = frame_id;
    ptStamped.header.stamp = cur_time;
    ptStamped.point.x = mCurGpsPose.mUtmEast;
    ptStamped.point.y = mCurGpsPose.mUtmNorth;
    ptStamped.point.z = mCurGpsPose.mAlt;

    point_pub.publish(ptStamped);
  }

  void publishPose()
  {
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = frame_id;
    poseStamped.header.stamp = cur_time;
    poseStamped.pose.position.x = mCurGpsPose.mUtmEast;
    poseStamped.pose.position.y = mCurGpsPose.mUtmNorth;
    poseStamped.pose.position.z = mCurGpsPose.mAlt;
    poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(std::atan2(mNextGpsPose.mUtmNorth - mCurGpsPose.mUtmNorth, mNextGpsPose.mUtmEast - mCurGpsPose.mUtmEast));

    pose_pub.publish(poseStamped);
  }

  void publishOdom()
  {
    double x = mCurGpsPose.mUtmEast;
    double y = mCurGpsPose.mUtmNorth;
    double z = mCurGpsPose.mAlt;

    double yaw = std::atan2(mNextGpsPose.mUtmNorth - mCurGpsPose.mUtmNorth, mNextGpsPose.mUtmEast - mCurGpsPose.mUtmEast);
    double lastYaw = std::atan2(mCurGpsPose.mUtmNorth - mLastGpsPose.mUtmNorth, mCurGpsPose.mUtmEast - mLastGpsPose.mUtmEast);


    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

    double dt = (cur_time - last_time).toSec();
    double dx = std::sqrt(std::pow(mCurGpsPose.mUtmEast - mLastGpsPose.mUtmEast,2) + std::pow(mCurGpsPose.mUtmNorth - mLastGpsPose.mUtmNorth,2))/dt;
    double dy = 0;

    double omega = yaw-lastYaw;
    omega = (omega >  M_PI) ? 2*M_PI - omega : omega;
    omega = (omega < -M_PI) ? omega + 2*M_PI : omega;
    omega /= dt;

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = cur_time;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = z;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = odom_trans.header.stamp;
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.twist.twist.linear.x = dx;
    odom.twist.twist.linear.y = dy;
    odom.twist.twist.angular.z = omega;

    //publish the message
    odom_pub.publish(odom);

    last_time = cur_time;
  }

  void start()
  {
    log_file.close();
    log_file.open(log_file_path);
    std::string line;
    std::getline(log_file, line);
    this->readLine(mLastGpsPose);
    this->readLine(mCurGpsPose);
    this->readLine(mNextGpsPose);

    if (firstTime == 0)
      firstTime = mLastGpsPose.mTime + timeDiff;
  }

  void next()
  {
    mLastGpsPose = mCurGpsPose;
    mCurGpsPose = mNextGpsPose;
    this->readLine(mNextGpsPose);
  }

  void run()
  {
    ros::Rate r(10);
    //Read a line
    this->start();
    while(EOFF)
    {
      //Publish position
      this->publishPoint();
      //Publish pose
      this->publishPose();
      //Odom pose
      this->publishOdom();


      ros::spinOnce();

      this->next();
      cur_time.fromSec(start_time.toSec() + (mCurGpsPose.mTime + timeDiff) - firstTime);
      ros::Time::sleepUntil(cur_time);
    }
  }

private:
  ros::NodeHandle n;
  ros::Publisher odom_pub, point_pub, pose_pub;
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time last_time, cur_time, start_time;
  double timeDiff, firstTime;
  GpsPose mLastGpsPose, mCurGpsPose, mNextGpsPose;
  std::string log_file_path, frame_id, child_frame_id;
  std::ifstream log_file;
  bool EOFF;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nmea2utm_node");

  NMEAPlayer mNMEAPlayer;

  mNMEAPlayer.run();

}
