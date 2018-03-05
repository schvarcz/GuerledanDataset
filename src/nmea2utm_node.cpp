#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <sstream>

class NMEAPlayer
{
public:

  NMEAPlayer () : first(true), log_file_path("gps_converted"), frame_id("odom"), child_frame_id("base_link")
  {
    ros::NodeHandle nodeLocal("~");
    frame_id = nodeLocal.param("frame_id", frame_id);
    child_frame_id = nodeLocal.param("child_frame_id", child_frame_id);
    log_file_path = nodeLocal.param("log_file", log_file_path);
    std::cout << log_file_path << std::endl;

    std::string ns = ros::this_node::getNamespace();

    odom_pub = n.advertise<nav_msgs::Odometry>(ns+"/odom", 50);
    point_pub = n.advertise<geometry_msgs::PointStamped>(ns+"/pointStamped", 50);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>(ns+"/poseStamped", 50);
    last_time = ros::Time::now();
    log_file.open(log_file_path);
    std::string line;
    std::getline(log_file, line);
  }

  bool readLine()
  {
    std::string line;
    bool EOFF = (std::getline(log_file, line)? true : false);

    if(!EOFF)
      return EOFF;

    std::stringstream part(line);
    std::string time, utmEast, utmNorth, lat, latH, longg, longH, fixQuali, nroSats, dilut, alt, altU, geoidH, geoidHU;

    std::getline(part,time,',');
    mTime = std::stod(time);

    std::getline(part,utmEast,',');
    mUtmEast = std::stod(utmEast);

    std::getline(part,utmNorth,',');
    mUtmNorth = std::stod(utmNorth);

    std::getline(part,lat,',');
    mLat = std::stod(lat);

    std::getline(part,latH,',');
    mLatH = latH;

    std::getline(part,longg,',');
    mLong = std::stod(longg);

    std::getline(part,longH,',');
    mLongH = longH;

    std::getline(part,fixQuali,',');
    mFixQuali = std::stod(fixQuali);

    std::getline(part,nroSats,',');
    mNroSats = std::stod(nroSats);

    std::getline(part,dilut,',');
    mDilut = std::stod(dilut);

    std::getline(part,alt,',');
    mAlt = std::stod(alt);

    std::getline(part,altU,',');
    mAltU = altU;

    std::getline(part,geoidH,',');
    mGeoidH = std::stod(geoidH);

    std::getline(part,geoidHU,',');
    mGeoidHU = geoidHU;

    return EOFF;
  }

  void publishPoint()
  {
    geometry_msgs::PointStamped ptStamped;
    ptStamped.header.frame_id = frame_id;
    ptStamped.header.stamp = ros::Time::now();
    ptStamped.point.x = this->mUtmEast;
    ptStamped.point.y = this->mUtmNorth;
    ptStamped.point.z = this->mAlt;

    point_pub.publish(ptStamped);
  }

  void publishPose()
  {
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = frame_id;
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.pose.position.x = this->mLong;
    poseStamped.pose.position.y = this->mLat;
    poseStamped.pose.position.z = this->mAlt;
//     poseStamped.pose.orientation = ;

    pose_pub.publish(poseStamped);
  }

  void publishOdom()
  {
    //        double yaw = pose2d.theta;

    //        double dt = (odom_msg.header.stamp - last_time).toSec();
    //        double dx = odom_msg.twist.twist.linear.x*dt*cos(yaw);
    //        double dy = odom_msg.twist.twist.linear.x*dt*sin(yaw);

    //        x += dx;
    //        y += dy;
    //        double omega = yaw-lastYaw;
    //        lastYaw = yaw;
    //        omega = (omega >  M_PI) ? 2*M_PI - omega : omega;
    //        omega = (omega < -M_PI) ? omega + 2*M_PI : omega;
    //        omega /= dt;

    //        //since all odometry is 6DOF we'll need a quaternion created from yaw
    //        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

    //        //first, we'll publish the transform over tf
    //        geometry_msgs::TransformStamped odom_trans;
    //        odom_trans.header.stamp = odom_msg.header.stamp;
    //        odom_trans.header.frame_id = "map";
    //        odom_trans.child_frame_id = "odom";

    //        odom_trans.transform.translation.x = x;
    //        odom_trans.transform.translation.y = y;
    //        odom_trans.transform.translation.z = 0.0;
    //        odom_trans.transform.rotation = odom_quat;

    //        //send the transform
    //        odom_broadcaster.sendTransform(odom_trans);

    //        //next, we'll publish the odometry message over ROS
    //        nav_msgs::Odometry odom;
    //        odom.header.stamp = odom_msg.header.stamp;
    //        odom.header.frame_id = "map";

    //        //set the position
    //        odom.pose.pose.position.x = x;
    //        odom.pose.pose.position.y = y;
    //        odom.pose.pose.position.z = 0.0;
    //        odom.pose.pose.orientation = odom_quat;

    //        //set the velocity
    //        odom.child_frame_id = "odom";
    //        odom.twist.twist.linear.x = odom_msg.twist.twist.linear.x;
    //        odom.twist.twist.linear.y = odom_msg.twist.twist.linear.y;
    //        odom.twist.twist.angular.z = omega;

    //        //publish the message
    //        odom_pub.publish(odom);

    //        last_time = odom_msg.header.stamp;
  }

  void run()
  {
    ros::Rate r(10);
    //Read a line
    while(this->readLine())
    {
        //Publish position
        this->publishPoint();

        //Publish pose
        this->publishPose();
        //Odom pose
        this->publishOdom();
        ros::spinOnce();
        r.sleep();
    }
  }

private:
  ros::NodeHandle n;
  ros::Publisher odom_pub, point_pub, pose_pub;
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time last_time;
  double mTime, mUtmEast, mUtmNorth, mLat, mLong, mFixQuali, mNroSats, mDilut, mAlt, mGeoidH;
  std::string mLatH, mLongH, mAltU, mGeoidHU;
  bool first;
  std::string log_file_path, frame_id, child_frame_id;
  std::ifstream log_file;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nmea2utm_node");

  NMEAPlayer mNMEAPlayer;

  mNMEAPlayer.run();

}
