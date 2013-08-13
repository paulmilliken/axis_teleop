#!/usr/bin/python

import rospy
from sensor_msgs.msg import Joy
from axis_camera.msg import Axis
from std_msgs.msg import Bool
from dynamic_reconfigure.server import Server
from axis_teleop.cfg import JoystickFunctionsConfig

class Teleop:
    def __init__(self):
        rospy.init_node('axis_ptz_speed_controller')
        self.initialiseVariables()
        self.pub = rospy.Publisher('cmd', Axis)
        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        self.pub_mirror = rospy.Publisher('mirror', Bool)
        self.pub_show_menu = rospy.Publisher('show_menu', Bool)
        
    def initialiseVariables(self):
        self.joy = None
        self.msg = Axis() # instantiate Axis message
        self.msg.autofocus = True # autofocus is on by default
        self.mirror = False
        self.mirror_already_actioned = False # to stop mirror flip-flopping
        self.show_menu = False
        self.show_menu_already_actioned = False # to stop flip-flopping
        # joystick axes [0..5] correspond to fwd, left, up, tilt right, 
        # tilt forwards, anticlockwise twist
        #self.joystick_function = {'pan_axis':3, 'tilt_axis':4, 'zoom_axis:5',
        #                        'mirror_button':1, 'menu_button':2}
        self.joystick_function = {}
        self.joystick_sensitivities = {'pan_axis':40.0, 'tilt_axis':60.0,
                                                            'zoom_axis':30.0}

    def spin(self):
        self.pub.publish(self.msg)
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.joy != None:
                self.createCmdMessage()
                self.createMirrorMessage()
                self.createShowMenuMessage()
            r.sleep()

    def createCmdMessage(self):
        '''Creates and publishes message to command the camera.  Spacenav axes
        are: [fwd, left, up, tilt_right, tilt_forward, twist_anticlockwise'''
        self.msg.pan = self.joy.axes[self.joystick_function['pan_axis']] * \
                                self.joystick_sensitivities['pan_axis']
        self.msg.tilt = self.joy.axes[self.joystick_function['tilt_axis']] * \
                                self.joystick_sensitivities['tilt_axis']
        self.msg.zoom = self.joy.axes[self.joystick_function['zoom_axis']] * \
                                self.joystick_sensitivities['zoom_axis']
        self.pub.publish(self.msg)

    def joy_callback(self, data):
        self.joy = data

    def createMirrorMessage(self):
        '''Creates and publishes message to indicate image should be mirrored'''
        if self.joy.buttons[self.joystick_function['mirror_button']]==1:
            if not self.mirror_already_actioned:
                self.mirror = not self.mirror
                self.mirror_already_actioned = True
        else:
            self.mirror_already_actioned = False
        self.pub_mirror.publish(Bool(self.mirror))

    def createShowMenuMessage(self):
        '''Creates and publishes message to show menu on teleop_view node'''
        if self.joy.buttons[self.joystick_function['show_menu_button']]==1:
            if not self.show_menu_already_actioned:
                self.show_menu = not self.show_menu
                self.show_menu_already_actioned = True
        else:
            self.show_menu_already_actioned = False
        self.pub_show_menu.publish(Bool(self.show_menu))

    def joystickFunctionsCallback(self, config, level):
        '''Defines what each axis and button of the joystick means'''
        self.joystick_function['pan_axis'] = config.pan_axis
        self.joystick_function['tilt_axis'] = config.tilt_axis
        self.joystick_function['zoom_axis'] = config.zoom_axis
        self.joystick_function['mirror_button'] = config.mirror_button
        self.joystick_function['show_menu_button'] = config.show_menu_button
        return(config)
    
if __name__ == "__main__":
    teleop = Teleop()
    srv_dynamic_reconfig = Server(JoystickFunctionsConfig,
                                            teleop.joystickFunctionsCallback)
    teleop.spin()
