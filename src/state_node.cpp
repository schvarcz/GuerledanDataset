#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <math.h>


// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
    ltrim(s);
    rtrim(s);
}

// trim from start (copying)
static inline std::string ltrim_copy(std::string s) {
    ltrim(s);
    return s;
}

// trim from end (copying)
static inline std::string rtrim_copy(std::string s) {
    rtrim(s);
    return s;
}

// trim from both ends (copying)
static inline std::string trim_copy(std::string s) {
    trim(s);
    return s;
}

class STATEPlayer
{
public:

  STATEPlayer () : log_file_path("logstate"), frame_id("odom"), child_frame_id("base_link"), timeDiff(0), firstTime(0), EOFF(false), thresholdDistance(0.5)
  {
    start_time = ros::Time::now();
    last_time = start_time;

    ros::NodeHandle nodeLocal("~");
    frame_id = nodeLocal.param("frame_id", frame_id);
    child_frame_id = nodeLocal.param("child_frame_id", child_frame_id);
    log_file_path = nodeLocal.param("log_file", log_file_path);
    timeDiff = nodeLocal.param("time_diff", timeDiff);
    firstTime = nodeLocal.param("first_time", firstTime);
    thresholdDistance = nodeLocal.param("thresholdDistance", thresholdDistance);

    std::string ns = ros::this_node::getNamespace();

    odom_pub = n.advertise<nav_msgs::Odometry>(ns+"/odom", 50);
    point_pub = n.advertise<geometry_msgs::PointStamped>(ns+"/pointStamped", 50);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>(ns+"/poseStamped", 50);
  }

  bool readLine(geometry_msgs::PoseStamped &rPose)
  {
    std::string line;
    EOFF = (std::getline(log_file, line)? true : false);

    if(!EOFF)
      return EOFF;

    std::stringstream part(line);
    std::string value;
    std::vector<double> values;

    while(std::getline(part,value,';'))
    {
      trim(value);
      if (!value.empty())
        values.push_back(std::stod(value));
    }

    rPose.header.stamp.fromSec(start_time.toSec() + (values[0] + timeDiff) - firstTime);
    rPose.pose.position.x = values[1];
    rPose.pose.position.y = values[2];
    rPose.pose.position.z = values[28];
    rPose.pose.orientation = tf::createQuaternionMsgFromYaw(((-values[29]+90)*2*M_PI)/360.);
    return EOFF;
  }

  void publishPoint()
  {
    geometry_msgs::PointStamped ptStamped;
    ptStamped.header.frame_id = frame_id;
    ptStamped.header.stamp = cur_time;
    ptStamped.point.x = mCurGpsPose.pose.position.x;
    ptStamped.point.y = mCurGpsPose.pose.position.y;
    ptStamped.point.z = mCurGpsPose.pose.position.z;

    point_pub.publish(ptStamped);
  }

  void publishPose()
  {
    geometry_msgs::PoseStamped poseStamped = mCurGpsPose;
    poseStamped.header.frame_id = frame_id;
    poseStamped.header.stamp = cur_time;
    pose_pub.publish(poseStamped);
  }

  double getDeltaYaw(geometry_msgs::PoseStamped mLastGpsPose, geometry_msgs::PoseStamped mCurGpsPose)
  {
      double yaw = tf::getYaw(mCurGpsPose.pose.orientation);
      double lastYaw = tf::getYaw(mLastGpsPose.pose.orientation);
      return getDeltaYaw(lastYaw, yaw);
  }

  double getDeltaYaw(double lastYaw, double yaw)
  {
      double omega = yaw-lastYaw;
      omega = (omega >  M_PI) ? 2*M_PI - omega : omega;
      omega = (omega < -M_PI) ? omega + 2*M_PI : omega;
      return omega;
  }

  double distance(geometry_msgs::PoseStamped mLastGpsPose, geometry_msgs::PoseStamped mCurGpsPose)
  {
    return sqrt(pow(mCurGpsPose.pose.position.y - mLastGpsPose.pose.position.y,2) + pow(mCurGpsPose.pose.position.x - mLastGpsPose.pose.position.x,2));
  }

  double linearVel(geometry_msgs::PoseStamped mLastGpsPose, geometry_msgs::PoseStamped mCurGpsPose)
  {
    double dist = distance(mLastGpsPose, mCurGpsPose);
    double yaw = tf::getYaw(mLastGpsPose.pose.orientation);
    double a[] = {
      (mCurGpsPose.pose.position.x - mLastGpsPose.pose.position.x) / dist,
      (mCurGpsPose.pose.position.y - mLastGpsPose.pose.position.y) / dist,
    };
    double b[] = {
      cos(yaw),
      sin(yaw),
    };
    if (a[0]*b[0]+a[1]*b[1] >= 0)
      return dist;
    return -dist;
  }

  void publishOdom()
  {
    double x = mCurGpsPose.pose.position.x;
    double y = mCurGpsPose.pose.position.y;
    double z = mCurGpsPose.pose.position.z;

    double yaw = tf::getYaw(mCurGpsPose.pose.orientation);
    double lastYaw = tf::getYaw(mLastGpsPose.pose.orientation);

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = mCurGpsPose.pose.orientation;

    double dt = (cur_time - last_time).toSec();
    double dx = linearVel(mLastGpsPose, mCurGpsPose)/dt;
//    dx = std::sqrt(std::pow(x - mLastGpsPose.pose.position.x,2) + std::pow(y - mLastGpsPose.pose.position.y,2))/dt;
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
    this->readLine(mLastFilePose);
    this->readLine(mCurFilePose);
    this->readLine(mNextFilePose);

    mLastGpsPose = mLastFilePose;
    mCurGpsPose  = mCurFilePose;
    mNextGpsPose = mNextFilePose;



    if (distance(mLastGpsPose, mCurGpsPose) > thresholdDistance)
      mLastGpsPose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(mCurGpsPose.pose.position.y - mLastGpsPose.pose.position.y, mCurGpsPose.pose.position.x - mLastGpsPose.pose.position.x));

    if (distance(mCurGpsPose, mNextGpsPose) > thresholdDistance)
      mCurGpsPose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(mNextGpsPose.pose.position.y - mCurGpsPose.pose.position.y, mNextGpsPose.pose.position.x - mCurGpsPose.pose.position.x));
    else
      mCurGpsPose.pose.orientation = tf::createQuaternionMsgFromYaw( tf::getYaw(mLastGpsPose.pose.orientation) + getDeltaYaw(mLastFilePose, mCurFilePose) );
//    if (firstTime == 0)
//      firstTime = mLastGpsPose.header.stamp.toSec() - start_time.toSec();
  }

  void next()
  {
    mLastGpsPose = mCurGpsPose;
    mCurGpsPose = mNextGpsPose;

    mLastFilePose = mCurFilePose;
    mCurFilePose = mNextGpsPose;

    this->readLine(mNextFilePose);
    mNextGpsPose = mNextFilePose;

    double newYaw = atan2(mNextGpsPose.pose.position.y - mCurGpsPose.pose.position.y, mNextGpsPose.pose.position.x - mCurGpsPose.pose.position.x);
    if ( (distance(mCurGpsPose, mNextGpsPose) > thresholdDistance) && (fabs(getDeltaYaw(tf::getYaw(mCurGpsPose.pose.orientation), newYaw)) < M_PI/3) )
      mCurGpsPose.pose.orientation = tf::createQuaternionMsgFromYaw(newYaw);
    else
      mCurGpsPose.pose.orientation = tf::createQuaternionMsgFromYaw( tf::getYaw(mLastGpsPose.pose.orientation) + getDeltaYaw(mLastFilePose, mCurFilePose) );
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
      cur_time = mCurGpsPose.header.stamp;
      ros::Time::sleepUntil(cur_time);
    }
  }

private:
  ros::NodeHandle n;
  ros::Publisher odom_pub, point_pub, pose_pub;
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time last_time, cur_time, start_time;
  double timeDiff, firstTime, thresholdDistance;
  geometry_msgs::PoseStamped mLastGpsPose, mCurGpsPose, mNextGpsPose;
  geometry_msgs::PoseStamped mLastFilePose, mCurFilePose, mNextFilePose;
  std::string log_file_path, frame_id, child_frame_id;
  std::ifstream log_file;
  bool EOFF;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_node");

  STATEPlayer mSTATEPlayer;

  mSTATEPlayer.run();

}
