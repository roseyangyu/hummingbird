#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from mavros_msgs.srv import CommandLong

service = None

class Buttons:
    X = 0
    A = 1
    B = 2
    Y = 3

class Command:
    ARM_DISARM = 400
    TAKEOFF = 22

def joyCallback(data):
    if data.buttons[Buttons.A]: # arm
        rospy.loginfo("A button pressed")
        try:
            service_stub = rospy.ServiceProxy(service, CommandLong)
            arm = True
            confirmation = False
            response = service_stub(False, Command.ARM_DISARM, confirmation, arm, 0, 0, 0, 0, 0, 0)
            print(response)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
    elif data.buttons[Buttons.B]: # disarm
        rospy.loginfo("B button pressed")
        try:
            service_stub = rospy.ServiceProxy(service, CommandLong)
            arm = False
            confirmation = False
            response = service_stub(False, Command.ARM_DISARM, confirmation, arm, 0, 0, 0, 0, 0, 0)
            print(response)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
    elif data.buttons[Buttons.Y]: # takeoff
        rospy.loginfo("Y button pressed")
        try:
            service_stub = rospy.ServiceProxy(service, CommandLong)
            confirmation = False
            response = service_stub(False, Command.TAKEOFF, confirmation, 0, 0, 0, 0, 0, 0, 0)
            print(response)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)

def main():
    rospy.init_node('hummingbird_joy', anonymous=True)
    global service
    service = rospy.get_param("~command_service")
    try:
        rospy.wait_for_service(service, timeout=5)
    except rospy.ROSException, e:
        rospy.logerr("Failed while waiting for service %s", service)
    rospy.Subscriber("/joy", Joy, joyCallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass