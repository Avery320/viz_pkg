#!/usr/bin/env python3

import rospy
import yaml
import sys
import os
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


# 全域變數
plane_x = 0.0
plane_y = 0.0
plane_z = 0.0
plane_a = 2.0
plane_b = 1.5
thickness = 0.005  # 固定厚度


def load_config():
    """載入簡單配置文件"""
    global plane_x, plane_y, plane_z, plane_a, plane_b
    try:
        import rospkg
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('viz_pkg')
        config_file = os.path.join(pkg_path, 'config', 'plane_config.yaml')

        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        plane_x = config['position']['x']
        plane_y = config['position']['y']
        plane_z = config['position']['z']
        plane_a = config['size']['a']
        plane_b = config['size']['b']

        rospy.loginfo(f"載入配置: 位置({plane_x}, {plane_y}, {plane_z}), 尺寸({plane_a}x{plane_b})")

    except Exception as e:
        rospy.logwarn(f"無法載入配置文件，使用預設值: {e}")


def pose_callback(msg):
    """接收位置更新"""
    global plane_x, plane_y, plane_z
    plane_x = msg.pose.position.x
    plane_y = msg.pose.position.y
    plane_z = msg.pose.position.z
    rospy.loginfo(f"位置更新: ({plane_x}, {plane_y}, {plane_z})")


def publish_marker():
    """發布平面標記 - 以位置為中心，x,y方向作為a,b大小，z固定-0.005"""
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    # 位置 - 以輸入位置為中心
    marker.pose.position.x = plane_x
    marker.pose.position.y = plane_y
    marker.pose.position.z = plane_z
    marker.pose.orientation.w = 1.0

    # 尺寸 - x,y方向作為a,b大小，z方向固定厚度
    marker.scale.x = abs(plane_a)  # X方向大小 (a)
    marker.scale.y = abs(plane_b)  # Y方向大小 (b)
    marker.scale.z = thickness     # Z方向厚度 (固定0.005)

    # 顏色 (藍色半透明)
    marker.color.r = 0.0
    marker.color.g = 0.5
    marker.color.b = 1.0
    marker.color.a = 0.7

    marker_pub.publish(marker)
def set_position(x, y, z):
    """手動設定位置"""
    global plane_x, plane_y, plane_z
    plane_x = float(x)
    plane_y = float(y)
    plane_z = float(z)
    rospy.loginfo(f"手動設定位置: ({plane_x}, {plane_y}, {plane_z})")


def set_size(a, b):
    """手動設定尺寸"""
    global plane_a, plane_b
    plane_a = float(a)
    plane_b = float(b)
    rospy.loginfo(f"手動設定尺寸: {plane_a} x {plane_b}")


if __name__ == '__main__':
    rospy.init_node('plane_visualizer')

    # 載入配置文件
    load_config()

    # ROS 參數覆蓋配置
    plane_a = rospy.get_param('~plane_a', plane_a)
    plane_b = rospy.get_param('~plane_b', plane_b)
    plane_x = rospy.get_param('~plane_x', plane_x)
    plane_y = rospy.get_param('~plane_y', plane_y)
    plane_z = rospy.get_param('~plane_z', plane_z)

    # 檢查指令行參數 (手動輸入功能)
    if len(sys.argv) == 4:
        # 格式: rosrun viz_pkg plane_visualizer.py x y z
        try:
            set_position(sys.argv[1], sys.argv[2], sys.argv[3])
        except ValueError:
            rospy.logwarn("指令行參數格式錯誤")
    elif len(sys.argv) == 6:
        # 格式: rosrun viz_pkg plane_visualizer.py x y z a b
        try:
            set_position(sys.argv[1], sys.argv[2], sys.argv[3])
            set_size(sys.argv[4], sys.argv[5])
        except ValueError:
            rospy.logwarn("指令行參數格式錯誤")

    # 發布者和訂閱者
    marker_pub = rospy.Publisher('/plane_marker', Marker, queue_size=1)
    pose_sub = rospy.Subscriber('/plane_pose', PoseStamped, pose_callback)

    # 定時發布
    timer = rospy.Timer(rospy.Duration(0.1), lambda event: publish_marker())

    rospy.loginfo(f"平面顯示節點啟動")
    rospy.loginfo(f"位置: ({plane_x}, {plane_y}, {plane_z})")
    rospy.loginfo(f"尺寸: {plane_a} x {plane_b}")
    rospy.loginfo(f"厚度: {thickness} (固定)")
    rospy.loginfo("手動輸入: rosrun viz_pkg plane_visualizer.py x y z [a b]")
    rospy.loginfo("位置更新: rostopic pub /plane_pose geometry_msgs/PoseStamped ...")

    rospy.spin()
