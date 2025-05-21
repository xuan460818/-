#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Empty

class MoveBaseSquare():

    def __init__(self):
        rospy.init_node('nav_pharmacy', anonymous=False)
        rospy.on_shutdown(self.shutdown)
     
        # Create a list to hold the target quaternions (orientations)
        # 创建一个列表，保存目标的角度数据
        quaternions = list()
         
        # First define the corner orientations as Euler angles
        # 定义四个顶角处机器人的方向角度（Euler angles:http://zh.wikipedia.org/wiki/%E6%AC%A7%E6%8B%89%E8%A7%92)
        #euler_angles = (0,pi/2, pi/2,-pi/2, -pi/2, pi/2,-pi/2,0,pi/4)
        euler_angles = (pi/2,pi/2, pi/2,-pi/2, -pi/2, -pi/2,-pi/2, 0,pi,0, 2.496,-1.008)
        # Then convert the angles to quaternions
        # 将上面的Euler angles转换成Quaternion的格式
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)
        
        # Create a list to hold the waypoint poses
        # 创建一个列表存储导航点的位置
        waypoints = list()
        waypoints.append(Pose(Point(1.42, 1.882, 0), quaternions[0]))      # //C
        waypoints.append(Pose(Point(0.637, 2.342, 0), quaternions[1]))      #//A
        waypoints.append(Pose(Point(1.515, 2.88, 0), quaternions[2]))      # //B
        waypoints.append(Pose(Point(-0.952, 0.946, 0), quaternions[3]))     # //4
        waypoints.append(Pose(Point(-1.715, 1.309, 0), quaternions[4]))    # //3
        waypoints.append(Pose(Point(-0.807, 1.744, 0), quaternions[5]))    # //2
        waypoints.append(Pose(Point(-1.604, 2.326, 0), quaternions[6]))    # //1
        waypoints.append(Pose(Point(0,0,0), quaternions[7]))    #起点
        waypoints.append(Pose(Point(-1.273,3.956,0), quaternions[8]))  #答题区   
        waypoints.append(Pose(Point(0.7,0,0), quaternions[9]))  #识别板1    
        waypoints.append(Pose(Point(0.67,3.599,0), quaternions[10]))  #配药区防撞点 
        waypoints.append(Pose(Point(-0.931,0.267,0), quaternions[11]))  #取药区防撞点    
        # Publisher to manually control the robot (e.g. to stop it)
        # 发布TWist消息控制机器人
        
        
        self.time0 = rospy.get_param("time0",1.0)
        self.time1 = rospy.get_param("time1",7.0)
        #self.time2 = rospy.get_param("time2",1.0)
        self.time3 = rospy.get_param("time3",1.0)
        #self.time4 = rospy.get_param("time4",1.0)
        self.time5 = rospy.get_param("time5",2.0)
        #self.time6 = rospy.get_param("time6",1.0)   #在配药区防撞点停顿1秒
        #self.time7 = rospy.get_param("time7",1.0)   #在取药区防撞点停顿1秒
        

        self.i = 0
        self.j = 0
        self.n = 1
        self.count = 9  #状态
        self.windows_ABC = 0
        self.windows_1234 = 4
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=12)
        self.cam_sub=rospy.Subscriber('/cam_send', Int32MultiArray, self.detect_result,queue_size=12)     #订阅检测的药品和窗口数组
        self.ram_result = [0,4]
        rospy.sleep(self.time0)
        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        rospy.loginfo("Starting navigation...")
        # 初始化一个计数器，记录到达的顶点号
        
        while(not rospy.is_shutdown()):#如果ros系统没有关机的话
            #有限状态机
            if(self.count == 9):#从起点到识别区
                rospy.loginfo("从起点到识别区")
                # Intialize the waypoint goal
                # 初始化goal为MoveBaseGoal类型
                goal = MoveBaseGoal()  
                # Use the map frame to define goal poses
                # 使用map的frame定义goal的frame id
                goal.target_pose.header.frame_id = 'map'
                # Set the time stamp to "now"
                # 设置时间戳
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[9]     #self.i
                
                if(self.move(goal) == True):
                    rospy.loginfo("到达识别区。。。。。。。。")
                    self.clear_costmaps_service()   #清除产生的偏移代价地图
                    self.count = 10
                    rospy.sleep(self.time1)    #给7秒钟识别时间
                    self.clear_costmaps_service()   #清除产生的偏移代价地图
                    
            elif(self.count == 10):#从识别区到配药区
                goal = MoveBaseGoal()  
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[self.windows_ABC]   
                if(self.move(goal) == True):
                    rospy.loginfo("到达字母区。。。。。。。。")
                    self.count = 11  
                    self.clear_costmaps_service()   #清除产生的偏移代价地图
                    # rospy.sleep(self.time2)      #在ABC的框里等待1秒钟，后面可以加播报
                    if self.windows_ABC == 0:
                        goal.target_pose.pose = waypoints[0]
                        os.system("play /home/EPRobot/robot_ws/src/pharmacy_pkg/music/peiC.mp3")
                    elif self.windows_ABC == 1:
                        goal.target_pose.pose = waypoints[1]
                        os.system("play /home/EPRobot/robot_ws/src/pharmacy_pkg/music/peiA.mp3")
                    elif self.windows_ABC == 2:
                        goal.target_pose.pose = waypoints[2]
                        os.system("play /home/EPRobot/robot_ws/src/pharmacy_pkg/music/peiB.mp3") 

    
            elif(self.count == 11):#从配药区到配药区防撞点
                rospy.loginfo("从配药区到答题区路上")
                goal = MoveBaseGoal()  
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[10]
                if(self.move(goal) == True):
                    rospy.loginfo("到达配药区防撞点。。。。。。。。")
                    self.count = 12  
                    self.clear_costmaps_service()   #清除产生的偏移代价地图
                    # rospy.sleep(self.time6) 
                    
            elif(self.count == 12):  #从配药区防撞点到答题区
                goal = MoveBaseGoal()  
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[8]     #self.i
                if(self.move(goal) == True):
                    rospy.loginfo("到达答题区防撞点")
                    self.count = 13
                    self.clear_costmaps_service()   #清除产生的偏移代价地图
                    rospy.sleep(self.time5) 
                    

            elif(self.count == 13):#从答题区到数字区
                rospy.loginfo("从答题区到数字区路上")
                goal = MoveBaseGoal() 
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[self.windows_1234]
                if(self.move(goal) == True):
                    rospy.loginfo("到达数字区。。。。。。。。")                    
                    self.count = 14
                    self.clear_costmaps_service()   #清除产生的偏移代价地图
                    # rospy.sleep(self.time4)     #在取药区等待1秒钟，后面可以加播报送药完成
                    if self.windows_1234 == 3:
                        goal.target_pose.pose = waypoints[3]
                        os.system("play /home/EPRobot/robot_ws/src/pharmacy_pkg/music/song4.mp3")
                    elif self.windows_1234 == 4:
                        goal.target_pose.pose = waypoints[4]
                        os.system("play /home/EPRobot/robot_ws/src/pharmacy_pkg/music/song3.mp3")
                    elif self.windows_1234 == 5:
                        goal.target_pose.pose = waypoints[5]
                        os.system("play /home/EPRobot/robot_ws/src/pharmacy_pkg/music/song2.mp3")   
                    elif self.windows_1234 == 6:
                        goal.target_pose.pose = waypoints[6]
                        os.system("play /home/EPRobot/robot_ws/src/pharmacy_pkg/music/song1.mp3") 
                        
            elif(self.count == 14):#从取药区到取药区防撞点
                rospy.loginfo("从取药区到取药区防撞点路上")
                goal = MoveBaseGoal()  
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[11]   #到达取药区防撞点。。。。。。。。
                if(self.move(goal) == True):
                    rospy.loginfo("到达配药区防撞点。。。。。。。。")
                    self.count = 9
                    self.clear_costmaps_service()   #清除产生的偏移代价地图
           
                     

    def move(self, goal):
            # Send the goal pose to the MoveBaseAction server
            # 把目标位置发送给MoveBaseAction的服务器
            self.move_base.send_goal(goal)
            # Allow 1 minute to get there
            # 设定1分钟的时间限制
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))
            # If we don't get there in time, abort the goal
            # 如果一分钟之内没有到达，放弃目标
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                # We made it!
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    return True
            return False        

    def detect_result(self,msg):
        self.ram_result = msg.data
        rospy.logwarn("self.ram_result: %s", self.ram_result)
        self.windows_ABC = self.ram_result[0]
        self.windows_1234 = self.ram_result[1]

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
