#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from mavros_msgs.srv import CommandInt

service = None
service_stub = None

class Buttons:
    X = 0
    A = 1
    B = 2
    Y = 3

class Command:
    ARM_DISARM = 400
    TAKEOFF = 22


def joyCallback(data):
    global service_stub
    if data.buttons[Buttons.A]: # arm
        rospy.loginfo("A button pressed")
        try:
            broadcast = False
            frame = 0
            command = Command.ARM_DISARM
            current = 0
            autocontinue = 0
            param1_arm = 1
            param2 = 0
            param3 = 0
            param4 = 0
            x = 0
            y = 0
            z = 0
            response = service_stub(broadcast, frame, command, current, autocontinue, param1_arm, param2, param3, param4, x, y, z)
            print(response)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
    elif data.buttons[Buttons.B]: # disarm
        rospy.loginfo("B button pressed")
        try:
            broadcast = False
            frame = 0
            command = Command.ARM_DISARM
            current = 0
            autocontinue = 0
            param1_arm = 0
            param2 = 0
            param3 = 0
            param4 = 0
            x = 0
            y = 0
            z = 0
            response = service_stub(broadcast, frame, command, current, autocontinue, param1_arm, param2, param3, param4, x, y, z)
            print(response)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
    elif data.buttons[Buttons.Y]: # takeoff
        rospy.loginfo("Y button pressed")
        try:
            broadcast = False
            frame = 0
            command = Command.TAKEOFF
            current = 0
            autocontinue = 0
            param1 = 0
            param2 = 0
            param3 = 0
            param4 = 0
            x = 0
            y = 0
            z = 0
            response = service_stub(broadcast, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z)
            print(response)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)

def main():
    rospy.init_node('hummingbird_joy', anonymous=True)
    global service
    global service_stub
    service = rospy.get_param("~command_service")
    try:
        rospy.wait_for_service(service, timeout=5)
        service_stub = rospy.ServiceProxy(service, CommandInt)
    except rospy.ROSException, e:
        rospy.logerr("Failed while waiting for service %s", service)
    rospy.Subscriber("/joy", Joy, joyCallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass