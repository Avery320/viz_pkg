#!/usr/bin/env python3
"""
位置發送節點
發送位置信息到平面可視化節點
"""

import rospy
import sys
import os
import yaml
from geometry_msgs.msg import PoseStamped

# 添加模組路徑
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))


def load_config():
    """載入配置文件獲取預設值

    Returns:
        tuple: (x, y, z) 位置坐標
    """
    config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'plane_config.yaml')
    
    try:
        with open(config_path, 'r', encoding='utf-8') as file:
            config = yaml.safe_load(file)
            position = config.get('position', {})
            x = position.get('x', 0.0)
            y = position.get('y', 0.0)
            z = position.get('z', 0.0)
            return x, y, z
    except Exception as e:
        rospy.logwarn(f"無法載入配置文件: {e}")
        return 0.0, 0.0, 0.0


def get_frame_id():
    """獲取座標系ID

    Returns:
        str: 座標系ID
    """
    return "map"


def send_position(x, y, z):
    """發送位置到平面顯示節點

    Args:
        x (float): X座標
        y (float): Y座標
        z (float): Z座標
    """
    # 建立發布者
    pose_pub = rospy.Publisher('/plane_pose', PoseStamped, queue_size=1)

    # 等待發布者連接建立
    rospy.sleep(1.0)

    # 建立 PoseStamped 訊息
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = get_frame_id()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.pose.position.x = float(x)
    pose_msg.pose.position.y = float(y)
    pose_msg.pose.position.z = float(z)
    pose_msg.pose.orientation.w = 1.0

    # 發布位置
    pose_pub.publish(pose_msg)
    rospy.loginfo(f"發送位置: ({x}, {y}, {z})")

    # 確保訊息發送完成
    rospy.sleep(0.5)


if __name__ == '__main__':
    rospy.init_node('send_position', anonymous=True)

    if len(sys.argv) == 4:
        # 使用命令行參數: rosrun viz_pkg send_position.py x y z
        try:
            x, y, z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
            send_position(x, y, z)
        except ValueError:
            rospy.logerr("參數格式錯誤，請使用數字")
            sys.exit(1)

    elif len(sys.argv) == 1:
        # 沒有參數，使用配置文件的預設值
        x, y, z = load_config()
        rospy.loginfo(f"使用配置文件預設位置: ({x}, {y}, {z})")
        send_position(x, y, z)

    else:
        rospy.logerr("使用方法:")
        rospy.logerr("  rosrun viz_pkg send_position.py x y z    # 指定位置")
        rospy.logerr("  rosrun viz_pkg send_position.py          # 使用配置文件預設位置")
        sys.exit(1)
