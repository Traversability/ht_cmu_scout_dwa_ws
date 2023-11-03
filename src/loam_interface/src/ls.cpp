void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn) {
  tf::StampedTransform transformTf;
  std::string sensorFrameId = laserCloudIn->header.frame_id;
  auto timeStamp = laserCloudIn->header.stamp;
  Eigen::Affine3d transformation;

  try {
    transformListener_ptr->waitForTransform("map", sensorFrameId, timeStamp, ros::Duration(1.0));
    transformListener_ptr->lookupTransform("map", sensorFrameId, timeStamp, transformTf);
    
    tf::transformTFToEigen(transformTf, transformation);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    cout << "Transform error" << std::endl;
    return;
  }

  // ... （读取并转换点云数据到Eigen或pcl格式）

  for (int i = 0; i < laserCloud->size(); i++) {
    Eigen::Vector3d point(laserCloud->points[i].x, laserCloud->points[i].y, laserCloud->points[i].z);
    Eigen::Vector3d transformedPoint = transformation * point;
    
    laserCloud->points[i].x = transformedPoint.x();
    laserCloud->points[i].y = transformedPoint.y();
    laserCloud->points[i].z = transformedPoint.z();
  }

  // 发布转换后的点云到地图坐标系
  sensor_msgs::PointCloud2 laserCloud2;
  pcl::toROSMsg(*laserCloud, laserCloud2);
  laserCloud2.header.stamp = laserCloudIn->header.stamp;
  laserCloud2.header.frame_id = "map";
  pubLaserCloudPointer->publish(laserCloud2);
}
