#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import tf2_py
import math



def pointcloud_callback(data):
    # 创建一个TF2转换器
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    # print('ht')

    try:
        # 获取点云数据的时间戳
        timestamp = data.header.stamp
        # 通过TF2转换器将点云数据从点云坐标系转换到地图坐标系
        transform = tf_buffer.lookup_transform("map", data.header.frame_id, rospy.Time(0), rospy.Duration(1))
        import sensor_msgs.point_cloud2 as pc2
        pc_transformed = pc2.transform_cloud(transform, data)
        # transformed_cloud = tf2_geometry_msgs.do_transform_cloud(data, transform)

        # 在这里，你可以对点云数据执行其他操作，然后将其发布到地图坐标系的话题
        # 这里只是简单地将其发布回点云话题，你可以根据需要进行修改
        
        pub.publish(transformed_cloud)
        print('ht1')


    except Exception as e:
        print(e)


def main():
    rospy.init_node('pointcloud_to_map_node', anonymous=True)
    rospy.Subscriber('/velodyne_points', PointCloud2, pointcloud_callback, queue_size=1)
    pub = rospy.Publisher('/transformed_pointcloud', PointCloud2, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()
