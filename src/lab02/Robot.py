#!/usr/bin/env python

import rospy, math, threading
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf



class Robot:
    def __init__(self):
        rospy.init_node('ZYang2_3002_Robot')
        self.rate = rospy.Rate(10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.nav_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.nav_to_pose)
        self.nav_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.targetX, self.targetY, self.targetR = 0, 0, 0
        self.nowX,self.nowY,self.nowR = 0,0,0
        self.targetR_ = None
        self.nav_goal = None
        self.twist_msg = Twist()
        self.basic_stop()
        self.tf_listener = tf.TransformListener()
        self.trans = None
        self.rotation = None
        self.target_dist = 0
        self.target_first_turn = 0
        self.is_driving_curve=True
        self.isArrived = True
        self.isArrived2 = True
        self.loop()

    def update_twist(self, straight, turn):
        turnlimit = math.pi/2
        straightlimit = 1
        if straight > 0.5:
            straight = 0.5
        if turn > turnlimit:
            turn = turnlimit
            straight = 0
        if turn < -turnlimit:
            turn = -turnlimit
            straight = 0
        # print straight,turn
        self.twist_msg.linear.x = float(straight)
        self.twist_msg.angular.z = float(turn)
        self.publisher.publish(self.twist_msg)

    def basic_Straight(self, speed):
        self.update_twist(speed, 0)

    def basic_Turn(self, speed):
        self.update_twist(0, speed)

    def basic_stop(self):
        self.update_twist(0,0)
        self.rate.sleep()

    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        """
        mark = self.nowR
        diff = (mark - angle)
        self.turn_absolute_fix (diff,mark)

    def driveStraight(self, distance):
        self.drive(distance, (self.nowX, self.nowY))

    def drive(self, distance, mark):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        """
        isArrived = False
        distance = float(distance)
        xmark, ymark = mark
        q = q = math.sqrt((self.nowX-xmark)**2+(self.nowY-ymark)**2)
        if q < distance/2:
            self.basic_Straight(((q / distance))/3  + 0.1)
        elif (q < distance) :
            self.basic_Straight((1 - ( q / distance))/3 + 0.05)
        else:
            isArrived = True
            self.basic_stop()
            rospy.loginfo("drived: "+str(round(distance,3)))
        # print q,distance
        self.rate.sleep()
        if not isArrived:
            self.drive(distance,mark)


    def in_range(self, range, num):
        return (num >= min(range)) and (num <= max(range))

    def get_range(self, num):
        num = int(num)
        if self.in_range([0, 90], num):
            return 4
        elif self.in_range([90, 180], num):
            return 3
        elif self.in_range([0, -90], num):
            return 1
        elif self.in_range([-180, -90], num):
            return 2

    def get_range_nav(self, x, y):
        if x == 0:
            x += 0.000000001
        if y == 0:
            y += 0.000000001
        if x > 0 and y > 0:
            return 4
        elif x > 0 and y < 0:
            return 1
        elif x < 0 and y > 0:
            return 3
        elif x < 0 and y < 0:
            return 2

    def turn_absolute_fix(self,target,mark):

        # print target,mark,'+++++'
        if target < -180:
            target = 360 + target
        if target > 180:
            target = 360 - target
        # print target,mark,'+++++'
        self.turn_absolute(target, mark)


    def turn_absolute(self, target, mark):
        isArrived = False
        now = self.nowR
        target_at_range = self.get_range(target)
        now_at_range = self.get_range(now)
        # if target_at_range is None:
        #     print target
        if now_at_range == target_at_range:
            if now < target:
                case = 'cw   <=90'
            else:
                case = 'ccw  <=90'

            if abs(now-target)<0.005:
                self.basic_stop()
                isArrived = True
                rospy.loginfo("turned: "+str(round(mark - target,3)))

        elif (abs(now_at_range - target_at_range) == 1) or (abs(now_at_range - target_at_range) == 3): #adj
            if now_at_range == 4 and target_at_range == 1:
                case = 'ccw   adj jump'
            elif now_at_range == 1 and target_at_range == 4:
                case = 'cw  adj jump'
            else:
                if now_at_range < target_at_range:
                    case = 'ccw adj'
                else:
                    case = 'cw  adj'
        else:
            i = abs(abs(now)+abs(target))
            if i < 180:
                case = 'cw   not adj'
            else:
                case = 'ccw  not adj'

        if not isArrived:
            if case[1] == 'w':
                if case[5] == '<':
                    self.basic_Turn(abs(now - target)/30 + 0.003)
                    # print(abs(now - target)/30 + 0.3, now)
                elif case[5] == 'a':
                    self.basic_Turn(abs(now - target)/50)
                else:
                    self.basic_Turn(abs(now - target)/50)
            elif case[1] == 'c':
                if case[5] == '<':
                    self.basic_Turn(-abs(now - target)/30 - 0.03)
                    # print(abs(now - target)/30 + 0.3, now)
                elif case[5] == 'a':
                    self.basic_Turn(-abs(now - target)/50)
                else:
                    self.basic_Turn(-abs(now - target)/50)

            self.rate.sleep()
            self.turn_absolute(target, mark)


    def translate_odom(self, odom):
        try:
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            _, _, r = euler_from_quaternion([odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w])
        except:
            x = odom.pose.position.x
            y = odom.pose.position.y
            _, _, r = euler_from_quaternion([odom.pose.orientation.x,odom.pose.orientation.y,odom.pose.orientation.z,odom.pose.orientation.w])
        r = r/math.pi*180
        return x, y, r


    def translate_tf(self, tf):

        x = tf[0][0]
        y = tf[0][1]
        _, _, r = euler_from_quaternion(tf[1])

        r = r/math.pi*180
        return x, y, r



    def nav_to_pose(self, goal):
        self.nav_goal = goal
        self.targetR_ = goal.pose.orientation
        self.targetX, self.targetY, self.targetR = self.translate_odom(goal)

        # i = []
        # for _ in range(10):
        #     i.append((self.target_first_turn, self.target_dist))
        #     self.rate.sleep()
        # print i.pop()
        
        if not self.is_driving_curve:
            self.turn_absolute_fix(self.target_first_turn+self.nowR, self.nowR)
            self.driveStraight(self.target_dist)
            self.turn_absolute_fix(self.targetR, self.nowR)
        else:
            self.isArrived = False
            self.isArrived2 = False
        rospy.loginfo("Phase 1")



    def odom_callback(self, odom):
        self.nowX, self.nowY, self.nowR = self.translate_odom(odom)



    def cap_curve_speed(self,linear,angular):
        return linear, angular

    def loop(self):
        while not rospy.is_shutdown():

            try:

                self.targetX, self.targetY, self.targetR = self.translate_odom(self.nav_goal)
                x,y,z,w = self.targetR_.x, self.targetR_.y, self.targetR_.z, self.targetR_.w
                self.tf_broadcaster.sendTransform((self.targetX, self.targetY, 0),
                                (x, y, z, w),
                                rospy.Time.now(),
                                'nav_goal',
                                "odom")
            except:
                pass
            try:
                (self.trans, self.rotation) = self.tf_listener.lookupTransform('/base_footprint', '/nav_goal', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self.target_first_turn = math.atan2(self.trans[1], self.trans[0])/math.pi*180
            self.target_dist = math.sqrt(self.trans[0] ** 2 + self.trans[1] ** 2)

            rdiff = abs(self.targetR - self.nowR)
            ldiff = self.target_dist


            if self.is_driving_curve and not self.isArrived2:
                
                if self.isArrived:
                    
                    a = (self.targetR - self.nowR) / 100
                    l = 0 
                    # print rdiff
                    if (rdiff < 0.05) and (ldiff < 0.05):
                        self.isArrived2 = True
                        a = 0

                        rospy.loginfo("Phase 2 Done. Arrived at x: " + str(self.targetX) + ' y: ' + str(self.targetY) + ' r: ' + str(self.targetR)) 
                else:
                    l = self.target_dist
                    a = self.target_first_turn /50
                    if l<0.005:
                        rospy.loginfo("Phase 1 Done, Start Phase 2")
                        self.isArrived = True
                        self.rate.sleep()

                self.update_twist(l,a)
            # print(self.target_first_turn, self.target_dist )
            self.rate.sleep()



if __name__ == '__main__':
    robot = Robot()
