#!/usr/bin/env python

import rospy, math, tf2_ros, argparse
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Robot:
    def __init__(self, isDriveingCurve):
        self.linear_max_speed =0.3
        rospy.init_node('ZYang2_3002_Robot')
        self.rate = rospy.Rate(50)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.nav_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.nav_to_pose)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_g = rospy.Publisher('/zyang/next_goal_dummy', TransformStamped, queue_size=10)
        self.target_list = [('stop', 0.0)]
        self.last_state = ''
        self.l, self.a = 0.0, 0.0
        self.twist_msg = Twist()
        self.isDriveingCurve = isDriveingCurve
        while not rospy.is_shutdown():
            self.loop()

    def update_twist(self):
        straight = float(self.l)
        turn = float(self.a)
        turn_limit = math.pi / 4
        straight_limit = self.linear_max_speed
        if straight > straight_limit:  # cap the speed
            straight = straight_limit
        if turn > turn_limit:
            turn = turn_limit
            straight = 0
        if turn < -turn_limit:
            turn = -turn_limit
            straight = 0
        self.twist_msg.linear.x = float(straight)
        self.twist_msg.angular.z = float(turn)
        self.publisher.publish(self.twist_msg)

    def rotate(self, angle):
        # angle = angle / 180 * math.pi
        try:
            next_goal = self.tf_buffer.lookup_transform('odom', 'base_footprint', rospy.Time())
            # rot = quaternion_from_euler(0, 0, angle)
            rot = euler_from_quaternion(
                [next_goal.transform.rotation.x, next_goal.transform.rotation.y, next_goal.transform.rotation.z,
                 next_goal.transform.rotation.w])

            rot1 = []
            rot1.append(rot[0])
            rot1.append(rot[1])
            rot1.append(rot[2] + angle)
            print rot1
            rr = quaternion_from_euler(rot1[0], rot1[1], rot1[2])
            next_goal.transform.rotation.x = rr[0]
            next_goal.transform.rotation.y = rr[1]
            next_goal.transform.rotation.z = rr[2]
            next_goal.transform.rotation.w = rr[3]

            self.publisher_g.publish(next_goal)
            self.rate.sleep()

        except Exception as e:
            print 'rotate: ', e
            pass

        pass

    def rotate_absolute(self, angle):
        # angle = angle / 180 * math.pi
        try:
            next_goal = self.tf_buffer.lookup_transform('odom', 'base_footprint', rospy.Time())
            next_goal1 = self.tf_buffer.lookup_transform('base_footprint', 'odom', rospy.Time())
            # rot = quaternion_from_euler(0, 0, angle)
            rot = euler_from_quaternion(
                [next_goal.transform.rotation.x, next_goal.transform.rotation.y, next_goal.transform.rotation.z,
                 next_goal.transform.rotation.w])
            rot1 = euler_from_quaternion(
                [next_goal1.transform.rotation.x, next_goal1.transform.rotation.y, next_goal1.transform.rotation.z,
                 next_goal1.transform.rotation.w])

            rot2 = []
            rot2.append(rot[0]+rot1[0])
            rot2.append(rot[1]+rot1[1])
            rot2.append(rot[2]+rot1[2] + angle)

            rr = quaternion_from_euler(rot2[0], rot2[1], rot2[2])
            next_goal.transform.rotation.x = rr[0]
            next_goal.transform.rotation.y = rr[1]
            next_goal.transform.rotation.z = rr[2]
            next_goal.transform.rotation.w = rr[3]

            self.publisher_g.publish(next_goal)
            self.rate.sleep()

        except Exception as e:
            print 'rotate: ', e
            pass

        pass

    def drive(self, distance):
        self.rate.sleep()
        pass

    def nav_to_pose(self, msg):
        self.target_list=[('stop',0.0)]
        try:
            buffer = []
            for i in range(10):
                buffer.append(self.tf_buffer.lookup_transform('base_footprint', 'nav_goal',
                                                              rospy.Time(0)))
                self.rate.sleep()
            trans = buffer.pop()
            target_first_turn = math.atan2(trans.transform.translation.y,
                                           trans.transform.translation.x)

            target_drive = math.sqrt(trans.transform.translation.x**2 + trans.transform.translation.y**2)

            _, _, target_second_turn = euler_from_quaternion(
                [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            self.target_list.append(('stop', 0.0))
            self.target_list.append(('rotate_absolute', target_second_turn))
            self.target_list.append(('stop', 0.0))
            if self.isDriveingCurve:
                self.target_list.append(('drive_curve', 'nav_goal'))
            else:
                self.target_list.append(('drive', target_drive))
                self.target_list.append(('stop', 0.0))
                self.target_list.append(('rotate', target_first_turn))
                self.target_list.append(('stop', 0.0))

        except:
            pass

        pass

    def do_target_list(self):
        try:
            (current_mode, parameter) = self.target_list[len(self.target_list) - 1]

            if current_mode == 'stop':
                print 'stop'
                self.l = 0.0
                self.a = 0.0
                self.target_list.pop()

            if current_mode == 'rotate':
                if self.last_state != current_mode:
                    print 'update rot'
                    self.rotate(parameter)
                    self.rate.sleep()
                try:
                    trans = self.tf_buffer.lookup_transform('base_footprint', 'dummy', rospy.Time())
                    _, _, self.a = euler_from_quaternion(
                        [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                         trans.transform.rotation.w])
                    self.l = 0.0
                except Exception as e:
                    print e
                    pass
                if abs(self.a) < 0.05 and (self.last_state == current_mode):
                    self.target_list.pop()
                    self.l = 0
                    self.a = 0
                    print 'finish turn', parameter


            if current_mode == 'rotate_absolute':
                if self.last_state != current_mode:
                    print 'update rot'
                    self.rotate_absolute(parameter)
                    self.rate.sleep()
                try:
                    trans = self.tf_buffer.lookup_transform('base_footprint', 'dummy', rospy.Time())
                    _, _, self.a = euler_from_quaternion(
                        [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                         trans.transform.rotation.w])
                    self.l = 0.0
                except Exception as e:
                    print e
                    pass
                if abs(self.a) < 0.05 and (self.last_state == current_mode):
                    self.target_list.pop()
                    self.l = 0
                    self.a = 0
                    print 'finish turn_a', parameter

            if current_mode == 'drive':
                if self.last_state != current_mode:
                    self.drive(parameter)
                    self.rate.sleep()

                try:
                    trans = self.tf_buffer.lookup_transform('base_footprint', 'dummy', rospy.Time())
                    self.l = parameter - math.sqrt(trans.transform.translation.x**2 + trans.transform.translation.y**2)

                    print '>>>',self.l
                except Exception as e:
                    print e
                    pass

                if self.l > self.linear_max_speed:
                    self.l = self.linear_max_speed/self.l
                print self.l
                if self.l < 0.05:
                    self.target_list.pop()
                pass



            if current_mode == 'drive_curve':
                if self.last_state != current_mode:
                    self.rate.sleep()

                try:
                    buffer = []
                    for i in range(10):
                        buffer.append(self.tf_buffer.lookup_transform('base_footprint', parameter, rospy.Time()))
                    trans = buffer.pop()


                    self.l = math.sqrt(trans.transform.translation.x**2 + trans.transform.translation.y**2)
                    self.a = math.atan2(trans.transform.translation.y,trans.transform.translation.x)


                    print '>>>',self.l
                except Exception as e:
                    print e
                    pass

                if self.l > self.linear_max_speed:
                    self.l = self.linear_max_speed/self.l
                print self.l
                if self.l < 0.01:
                    self.target_list.pop()
                pass

            self.last_state = current_mode
        except:
            pass

        self.rate.sleep()

    def loop(self):
        self.do_target_list()
        self.update_twist()
        self.rate.sleep()


if __name__ == '__main__':

    # parser = argparse.ArgumentParser()
    # parser.add_argument("curve", help="curve driving enabled", type=int)
    # args = parser.parse_args()
    # print args.curve
    # if args.curve == 1:
    robot = Robot(True)
    # else:
    #     robot = Robot(False)
