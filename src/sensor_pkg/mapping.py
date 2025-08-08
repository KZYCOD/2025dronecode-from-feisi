import cv2
import numpy as np
import rospy
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import TransformStamped
import tf
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
    
def odom_cb(msg:Odometry):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "odom1", #测试直接从lidar 到 map
                     "map")



def pointcloud_cb(msg):
    try:
        # 获取 lidar -> map 的变换
        (trans, rot) = tf_listener.lookupTransform("map", "lidar", rospy.Time(0))

        # 将四元数转换为旋转矩阵
        transform_matrix = tf.transformations.quaternion_matrix(rot)
        transform_matrix[0:3, 3] = trans  # 添加平移部分

        # 转换点云
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        transformed_points = []
        for point in points:
            # 将点从 lidar 坐标系转换到 map 坐标系
            point_lidar = np.array([point[0], point[1], point[2], 1.0])  # 齐次坐标
            point_map = np.dot(transform_matrix, point_lidar)  # 应用变换
            transformed_points.append([point_map[0], point_map[1], point_map[2]])

        # 创建新的点云消息
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"  # 设置新的坐标系为 map
        transformed_cloud = point_cloud2.create_cloud_xyz32(header, transformed_points)

        # 发布转换后的点云
        transformed_cloud_pub.publish(transformed_cloud)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn("TF transform error: %s", e)

if __name__ == "__main__":
    rospy.init_node("pointcloud_transformer")

    # TF 监听器
    tf_listener = tf.TransformListener()

    # 订阅原始点云
    rospy.Subscriber("/rflysim/sensor0/mid360_lidar", PointCloud2, pointcloud_cb)
    rospy.Subscriber("/rflysim/uav1/local/odom", Odometry, odom_cb)

    # 发布转换后的点云
    transformed_cloud_pub = rospy.Publisher("/map/points", PointCloud2, queue_size=10)

    rospy.spin()
