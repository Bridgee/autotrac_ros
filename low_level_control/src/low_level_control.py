#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import time

class my_Servo():
    def __init__(self, min=4750, max=7000, center=5870):
        self.value      = 0.0
        self.value_out  = center
        self._center    = center
        self._min       = min
        self._max       = max

    # from twist value to pwm value
    def get_value_out(self, value_in):
        self.value      = value_in
        self.value_out  = int(self._center + value_in*500)
        print 'Servo: ', self.value_out
        if self.value_out < self._min:
            self.value_out = self._min
        if self.value_out > self._max:
            self.value_out = self._max
        print 'Servo_final: ', self.value_out
        return(self.value_out)

class my_Motor():
    def __init__(self, min=1007, max=10280):
        self.value      = 0.0
        self.value_out  = min
        self._min       = min
        self._max       = max
        self._brake     = 0

    # from twist value to pwm value
    def get_value_out(self, value_in):
        self.value      = value_in
        if value_in <= 0:
            self.value_out = int(0)
        else:
            self.value_out = int(self._min + value_in*1000)
            print 'Motor: ', self.value_out
            if self.value_out < self._min:
                self.value_out = self._min
            if self.value_out > self._max:
                self.value_out = self._max
        print 'Motor_final: ', self.value_out
        return(self.value_out)

class AT_LLC():
    def __init__(self):
        rospy.loginfo('Setting Up the Node...')
        rospy.init_node('at_llc')

        # dict of actuators
        self.actuators = {}
        self.actuators['throttle'] = my_Motor()
        self.actuators['steering'] = my_Servo()
        rospy.loginfo('Actuators Initialized')

        self._pwm_msg = Int32MultiArray()

        # Create publisher
        self.ros_pub_pwm = rospy.Publisher('/command', Int32MultiArray, queue_size=1)
        rospy.loginfo('Publisher Initialized')

        # Create subscriber
        self.ros_sub_twist = rospy.Subscriber('/turtle1/cmd_vel', Twist, self.cmd_to_act)
        rospy.loginfo('Subscriber Initialized')

        # Get the last time got commands
        self._last_time_cmd_rcv = time.time()
        self._timeout_s = 5

        rospy.loginfo('Initialization Complete!!!')

    # Twist Callback
    def cmd_to_act(self, message):
        print 'Callback...'
        self._last_time_cmd_rcv = time.time()
        
        if message.linear.x != 0:
            self.actuators['throttle'].get_value_out(message.linear.x) # twist to pwm value
        if message.angular.z !=0:
            self.actuators['steering'].get_value_out(message.angular.z) # twist to pwm value

        rospy.loginfo('Got a command v = %2.1f  s = %2.1f'%(message.linear.x, message.angular.z))

    def idle_control(self):
        print 'Idle Control...'
        self.actuators['throttle'].get_value_out(-1)
        self.actuators['steering'].get_value_out(0)
        rospy.loginfo('Setting actutors to idle')

    def send_pwm_msg(self):
        self._pwm_msg.data = [-1,
                              -1,
                              -1,
                              -1,
                              -1,
                              -1,
                              -1,
                              -1,
                              -1,
                              -1,
                              -1,
                              -1,
                              -1,
                              -1,
                              self.actuators['steering'].value_out,
                              self.actuators['throttle'].value_out]
        self.ros_pub_pwm.publish(self._pwm_msg)

    @property
    def is_controller_connected(self):
        return(time.time() - self._last_time_cmd_rcv < self._timeout_s)

    def run(self):
        rate = rospy.Rate(500)
        while not rospy.is_shutdown():
            if not self.is_controller_connected:
                print 'lost control'
                self.idle_control()
            self.send_pwm_msg()
            rate.sleep()
                

if __name__ == "__main__":
    autotrac_llc = AT_LLC()
    autotrac_llc.run()



























