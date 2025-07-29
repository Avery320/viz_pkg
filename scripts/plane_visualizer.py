#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class PlaneVisualizer:
    """平面視覺化器 - 發布視覺化標記和碰撞物體"""
    
    def __init__(self):
        # 初始化參數
        self.plane_x = 0.0
        self.plane_y = 0.0
        self.plane_z = 0.0
        self.plane_a = 2.0
        self.plane_b = 1.5
        self.plane_c = 0.005  # 固定厚度
        
        # 處理命令行參數
        self._parse_command_line()
        
        # 初始化發布者
        self.marker_pub = rospy.Publisher('/plane_marker', Marker, queue_size=1)
        self.collision_pub = rospy.Publisher('/collision_object', CollisionObject, queue_size=1)
        self.pose_sub = rospy.Subscriber('/plane_pose', PoseStamped, self._pose_callback)
        
        # 設置定時器
        rospy.Timer(rospy.Duration(0.1), lambda event: self._publish_marker())
        rospy.Timer(rospy.Duration(1.0), lambda event: self._publish_collision_object())
        
        self._log_info()
    
    def _parse_command_line(self):
        """解析命令行參數"""
        if len(sys.argv) == 6:
            try:
                self.plane_x = float(sys.argv[1])
                self.plane_y = float(sys.argv[2])
                self.plane_z = float(sys.argv[3])
                self.plane_a = float(sys.argv[4])
                self.plane_b = float(sys.argv[5])
                rospy.loginfo(f"命令行設定: 位置({self.plane_x}, {self.plane_y}, {self.plane_z}), 尺寸({self.plane_a}x{self.plane_b})")
            except ValueError:
                rospy.logwarn("命令行參數格式錯誤，使用預設值")
        elif len(sys.argv) > 1:
            rospy.logwarn("需要5個參數: x y z a b")
            rospy.logwarn("範例: rosrun viz_pkg plane_visualizer.py 1.0 2.0 0.5 3.0 2.5")
    
    def _pose_callback(self, msg):
        """接收位置更新"""
        self.plane_x = msg.pose.position.x
        self.plane_y = msg.pose.position.y
        self.plane_z = msg.pose.position.z
        rospy.loginfo(f"位置更新: ({self.plane_x}, {self.plane_y}, {self.plane_z})")
    
    def _publish_marker(self):
        """發布視覺化標記"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # 位置 - 偏移到中心點
        marker.pose.position.x = self.plane_x + self.plane_a / 2.0
        marker.pose.position.y = self.plane_y + self.plane_b / 2.0
        marker.pose.position.z = self.plane_z + self.plane_c / 2.0
        marker.pose.orientation.w = 1.0
        
        # 尺寸
        marker.scale.x = self.plane_a
        marker.scale.y = self.plane_b
        marker.scale.z = self.plane_c
        
        # 顏色 (藍色半透明)
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.7
        
        self.marker_pub.publish(marker)
    
    def _publish_collision_object(self):
        """發布碰撞物體"""
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.header.stamp = rospy.Time.now()
        collision_object.id = "plane_collision"
        collision_object.operation = CollisionObject.ADD
        
        # 創建實體形狀
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [self.plane_a, self.plane_b, self.plane_c]
        
        # 設定位置
        pose = Pose()
        pose.position.x = self.plane_x + self.plane_a / 2.0
        pose.position.y = self.plane_y + self.plane_b / 2.0
        pose.position.z = self.plane_z + self.plane_c / 2.0
        pose.orientation.w = 1.0
        
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        
        self.collision_pub.publish(collision_object)
        rospy.loginfo(f"發布碰撞物體: 起始點({self.plane_x}, {self.plane_y}, {self.plane_z}), 尺寸({self.plane_a}x{self.plane_b}x{self.plane_c})")
    
    def _log_info(self):
        """輸出啟動信息"""
        rospy.loginfo("平面顯示節點啟動")
        rospy.loginfo(f"起始點: ({self.plane_x}, {self.plane_y}, {self.plane_z})")
        rospy.loginfo(f"尺寸: {self.plane_a} x {self.plane_b}")
        rospy.loginfo(f"厚度: {self.plane_c} (固定)")
        rospy.loginfo("碰撞物體發布到: /collision_object")
        rospy.loginfo("命令行啟動: rosrun viz_pkg plane_visualizer.py x y z a b")
        rospy.loginfo("範例: rosrun viz_pkg plane_visualizer.py 1.0 2.0 0.5 3.0 2.5")
        rospy.loginfo("位置更新: rostopic pub /plane_pose geometry_msgs/PoseStamped ...")


if __name__ == '__main__':
    rospy.init_node('plane_visualizer')
    visualizer = PlaneVisualizer()
    rospy.spin()
