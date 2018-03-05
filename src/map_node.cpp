#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

using namespace std;

class MAPLoader
{
public:

  MAPLoader () : log_file_path("map.pts"), frame_id("helius")
  {
    minX = numeric_limits<double>::min(); maxX = numeric_limits<double>::max();
    minY = numeric_limits<double>::min(); maxY = numeric_limits<double>::max();
    minZ = numeric_limits<double>::min(); maxZ = numeric_limits<double>::max();

    ros::NodeHandle nodeLocal("~");
    frame_id = nodeLocal.param("frame_id", frame_id);
    log_file_path = nodeLocal.param("log_file", log_file_path);
    minX = nodeLocal.param("minX", minX);
    maxX = nodeLocal.param("maxX", maxX);
    minY = nodeLocal.param("minY", minY);
    maxY = nodeLocal.param("maxY", maxY);
    minZ = nodeLocal.param("minZ", minZ);
    maxZ = nodeLocal.param("maxZ", maxZ);
    std::cout << log_file_path << std::endl;

    std::string ns = ros::this_node::getNamespace();

    pc_pub = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1);

    log_file.open(log_file_path);
  }

  sensor_msgs::PointCloud loadMap()
  {
    string line, xs, ys, zs;
    double x, y, z;

    sensor_msgs::PointCloud pc_msg;
    geometry_msgs::Point32 point;
    while(getline(log_file, line))
    {
      stringstream sline(line);
      getline(sline, xs, ' ');
      getline(sline, ys, ' ');
      getline(sline, zs, ' ');
      x = stod(xs);
      y = stod(ys);
      z = -stod(zs);

      if ((minX < x) && (x < maxX) &&
          (minY < y) && (y < maxY) &&
          (minZ < z) && (z < maxZ))
      {
        point.x = x;
        point.y = y;
        point.z = z;
        pc_msg.points.push_back(point);
      }
    }
    return pc_msg;
  }

  void run()
  {
    sensor_msgs::PointCloud pc_msg = this->loadMap();

    ros::Rate r(0.1);

    while(ros::ok())
    {

      ros::Time current_time = ros::Time::now();
      std::cout << current_time << std::endl;

      pc_msg.header.stamp = current_time;
      pc_msg.header.frame_id = frame_id+"_pointcloud";
      pc_pub.publish(pc_msg);

      geometry_msgs::TransformStamped pc_trans;
      pc_trans.header.stamp = current_time;
      pc_trans.header.frame_id = frame_id;
      pc_trans.child_frame_id = frame_id+"_pointcloud";

      pc_trans.transform.translation.x = 0.0;
      pc_trans.transform.translation.y = 0.0;
      pc_trans.transform.translation.z = 0.0;
      pc_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
      tf_broadcaster.sendTransform(pc_trans);
      ros::spinOnce();
      r.sleep();
    }
  }


private:
  ros::NodeHandle n;
  ros::Publisher pc_pub;
  tf::TransformBroadcaster tf_broadcaster;

  std::string log_file_path, frame_id;
  std::ifstream log_file;

  double minX, maxX, minY, maxY, minZ, maxZ;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_node");

  MAPLoader mMAPLoader;
  mMAPLoader.run();

//  ros::spin();

  return 0;
}
