#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

// #include <tf/transform_listener.h>
// #include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

using namespace std;

const double PI = 3.1415926;

string stateEstimationTopic = "/integrated_to_init";
string registeredScanTopic = "/velodyne_cloud_registered";
bool flipStateEstimation = true;
bool flipRegisteredScan = true;
bool sendTF = true;
bool reverseTF = false;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());

nav_msgs::Odometry odomData;
tf::StampedTransform odomTrans;
ros::Publisher *pubOdometryPointer = NULL;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
ros::Publisher *pubLaserCloudPointer = NULL;


void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  odomData = *odom;

  if (flipStateEstimation) {
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

    pitch = -pitch;
    yaw = -yaw;

    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = odom->pose.pose.position.z;
    odomData.pose.pose.position.y = odom->pose.pose.position.x;
    odomData.pose.pose.position.z = odom->pose.pose.position.y;
  }

  // publish odometry messages
  odomData.header.frame_id = "map";
  odomData.child_frame_id = "sensor";
  pubOdometryPointer->publish(odomData);

  // publish tf messages
  odomTrans.stamp_ = odom->header.stamp;
  odomTrans.frame_id_ = "map";
  odomTrans.child_frame_id_ = "sensor";
  odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
  odomTrans.setOrigin(tf::Vector3(odomData.pose.pose.position.x, odomData.pose.pose.position.y, odomData.pose.pose.position.z));

  if (sendTF) {
    if (!reverseTF) {
      tfBroadcasterPointer->sendTransform(odomTrans);
    } else {
      tfBroadcasterPointer->sendTransform(tf::StampedTransform(odomTrans.inverse(), odom->header.stamp, "sensor", "map"));
    }
  }
}

// 函数没用
// void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
// {
//   try {
//         // 创建一个tf2_ros::Buffer和tf2_ros::TransformListener
//         tf2_ros::Buffer tfBuffer;
//         tf2_ros::TransformListener tfListener(tfBuffer);

//         // 获取点云消息中的时间戳
//         ros::Time cloud_time = msg->header.stamp;
        
//         // 使用tf2_ros::Buffer来获取点云到地图的变换
//         geometry_msgs::TransformStamped transform;
//         cout<<msg->header.frame_id<<endl;
//         transform = tfBuffer.lookupTransform("hunter_se/odom", msg->header.frame_id, cloud_time);

//         // 将点云数据转换到地图坐标系
//         sensor_msgs::PointCloud2 transformed_cloud;
//         // pcl_ros::transformPointCloud(target_frame, source_cloud, transformed_cloud, tf_listener);

//         // tfBuffer.transform(*msg, transformed_cloud, "map");

//         // transformed_cloud.header.frame_id = "map"; // 设置新的坐标系

//         pubLaserCloudPointer->publish(transformed_cloud);
//     } catch (tf2::TransformException& ex) {
//         cout<<("Transform lookup failed: %s", ex.what())<<endl;
//         ROS_WARN("Transform lookup failed: %s", ex.what());
//     }
  
// }

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  tf::TransformListener transformListener_;

  tf::StampedTransform transformTf;
  std::string sensorFrameId = msg->header.frame_id;
  auto timeStamp = msg->header.stamp;
  Eigen::Affine3d transformationSensorToMap;
  try {
    transformListener_.waitForTransform("map", sensorFrameId, timeStamp, ros::Duration(1.0));
    transformListener_.lookupTransform("map", sensorFrameId, timeStamp, transformTf);
    poseTFToEigen(transformTf, transformationSensorToMap);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "loamInterface");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("stateEstimationTopic", stateEstimationTopic);
  nhPrivate.getParam("registeredScanTopic", registeredScanTopic);
  nhPrivate.getParam("flipStateEstimation", flipStateEstimation);
  nhPrivate.getParam("flipRegisteredScan", flipRegisteredScan);
  nhPrivate.getParam("sendTF", sendTF);
  nhPrivate.getParam("reverseTF", reverseTF);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> (stateEstimationTopic, 1, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> (registeredScanTopic, 1, laserCloudHandler);

  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> ("/state_estimation", 5);
  pubOdometryPointer = &pubOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/registered_scan", 5);
  pubLaserCloudPointer = &pubLaserCloud;

  // tf::TransformListener transformListener;
  // transformListener_ptr = &transformListener;

  ros::spin();

  return 0;
}
