#!/usr/bin/env python
import rospy
# # *for string, bool, empty
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# from tf import transformations
# from std_srvs.srv import *
# import time
# from move_base_msgs.msg import MoveBaseActionGoal
from debugpy import listen
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

# * Program has to do three main things:
# * - 1) autonomously reach a x,y coordinate inserted by the user
# * - 2) let the user drive the robot with the keyboard
# * - 3) let the user drive the robot assisting them to avoid collisions
# -----------------------------------------------

# * 1A) the move_base pakage requires goal to be sent to the
# * topic move_base/goal, by sending a message of type
# * move_base_msgs/MoveBaseActionGoal (check the PDF for the correct data page6)
# -----------------------------------------------

# * Regarding 2 and 3), you can rely on the teleop_twist_keyboard seen
# * in class. However, in case 3), the cmd_vel may need to be corrected
# * when the user is going to crash into obstacles. Carefully consider the
# * architecture of the system.
# -----------------------------------------------
# * Concerning 3), the robot:
# * - should not go forward if there is an obstacle in the front
# * - should not turn left/right if there are obstacles on the
# * left/right
# TODO
# TODO- Main as UI that gets input and sends it to move_base/goal
# TODO (implement UI after)
# TODO- Main that rephrases data form teleop_twist_keyboard
# TODO  to move_base (remap teleop topic and send it to move_base)
# TODO- After implementing teleop remap make the function
# TODO consider obstacles form laser scanner and MAP

regions = {
    'right':    0,
    'fright':   0,
    'front':    0,
    'fleft':    0,
    'left':     0,
}
globalVelocity = None
velocityToSend = None
pubToDrive = None
pubFromAssisted = None


def autoGoalDriveCallback(msg):
    print("autoGoalDriveCallback running!")
    print("This is the recievied message : ", msg.data)
    movebase_client(msg.data[0], msg.data[1])
    print("Action client returned correctly")


def manualDriveCallback(msg):
    global velocityToSend
    print("manualDriveCallback running!")
    if msg == True:
        velocityToSend = globalVelocity
        pubToDrive.publish(velocityToSend)


def assistedDriveCallback(msg):
    # TODO algo for avoiding obstacles
    global velocityToSend
    print("assistedDriveCallback running!")
    if msg == True:
        velocityToSend = globalVelocity
        pubToDrive.publish(velocityToSend)


# def clbk_laser(msg):
#     regions = {
#         'right':  min(min(msg.ranges[0:143]), 10),
#         'fright': min(min(msg.ranges[144:287]), 10),
#         'front':  min(min(msg.ranges[288:431]), 10),
#         'fleft':  min(min(msg.ranges[432:575]), 10),
#         'left':   min(min(msg.ranges[576:719]), 10),
#     }


def makeDrivingGreatAgain(msg):
    global globalVelocity
    globalVelocity = Twist()
    globalVelocity = msg


def movebase_client(xUI, yUI):
    print("running move_base_Client")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.orientation.w = 1.0
    goal.target_pose.pose.position.x = xUI
    goal.target_pose.pose.position.y = yUI

    client.send_goal(goal)
    print("Waiting for the action client result")
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        print("Finished")
        return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('master_Robot')
        print("MASTER::THIS IS THE TEST AFTER SUBS")

        rospy.Subscriber("autonomousCoords", Float32MultiArray,
                         autoGoalDriveCallback)
        rospy.Subscriber("manualDriveCheck", Bool,
                         manualDriveCallback)
        rospy.Subscriber("assistedDriveCheck", Bool,
                         assistedDriveCallback)
        rospy.Subscriber("remmaped_cmd_vel", Twist,
                         makeDrivingGreatAgain)
        pubToDrive = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        result = True
        print("MASTER::THIS IS THE TEST AFTER SUBS")
        # result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
