#!/usr/bin/env python
import statistics
import rospy
# # *for string, bool, empty
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import os
import curses
import sys
import builtins
from statistics import mean

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
# TODO- Main as UI that gets input and sends it to move_base/goal   DONE
# TODO (implement UI after)
# TODO- Main that rephrases data form teleop_twist_keyboard         DONE
# TODO  to move_base (remap teleop topic and send it to move_base)  DONE
# TODO- After implementing teleop remap make the function           DONE
# TODO consider obstacles form laser scanner and MAP

regions = {
    'right':    0,
    'fright':   0,
    'front':    0,
    'fleft':    0,
    'left':     0,
}

meanOfRanges = None

globalVelocity = Twist()
velocityToSend = Twist()

# globalVelocity.linear.x = 0
# globalVelocity.linear.y = 0
# globalVelocity.linear.z = 0
# globalVelocity.angular.x = 0
# globalVelocity.angular.y = 0
# globalVelocity.angular.z = 0

remmapedListner = None
pubToDrive = None
pubFromAssisted = None
# ? Params
assitanceThreshold = None
timeoutROS = None

r = None
menu_options = {
    1: 'Autonomously reach a x,y coordinate inserted by the user',
    2: 'Drive the robot with the keyboard',
    3: 'Drive the robot with assistance to avoid collisions',
    4: 'Exit',
    5: "IT'S TIME TO STOP",
}


def prRed(skk): print("\033[91m {}\033[00m" .format(skk))
def prGreen(skk): print("\033[92m {}\033[00m" .format(skk))
def prYellow(skk): print("\033[93m {}\033[00m" .format(skk))
def prLightPurple(skk): print("\033[94m {}\033[00m" .format(skk))
def prPurple(skk): print("\033[95m {}\033[00m" .format(skk))
def prCyan(skk): print("\033[96m {}\033[00m" .format(skk))
def prLightGray(skk): print("\033[97m {}\033[00m" .format(skk))
def prBlack(skk): print("\033[98m {}\033[00m" .format(skk))


def print_menu():
    # TODO drawing UI one at the time?
    # clear_screen_seq = b''

    # if sys.stdout.isatty():
    #     curses.setupterm()
    #     clear_screen_seq = curses.tigetstr('clear')
    # print("\033c")
    # print("\033c", end="")
    os.system('clear')
    print("\033[96m")
    for key in menu_options.keys():
        # builtins.print(key, '--', menu_options[key])
        print(key, '--', menu_options[key])
    print("\033[00m")
    # time.sleep(0.1)

    # scrn = curses.initscr()
    # # scrn.erase()
    # scrn.erase()
    # scrn.refresh()
    # curses.endwin()


def autoGoalDriveCallback(msg):
    prYellow("Master:: autoGoalDriveCallback running!")
    print("\033[93m This is the recievied message : ", msg.data, "\033[00m")
    movebase_client(msg.data[0], msg.data[1])
    prYellow("Action client returned correctly")
    print_menu()


def makeDrivingGreatAgain(msg):
    prLightPurple("\nMaster:: detected teleop input")
    global globalVelocity
    globalVelocity.linear = msg.linear
    globalVelocity.angular = msg.angular
    r.sleep()

    # print("This is the message", msg)
    # print("This is the globalbelocity", globalVelocity)


def manualDriveCallback(msg):
    global globalVelocity
    if (msg.data == True):
        prYellow("Master:: manualDriveCallback running!")
        prCyan("Control of the robot is done through seperate console 'TeleopKey!'")
    while(msg.data == True):
        pubToDrive.publish(globalVelocity)
        r.sleep()
    prYellow("Master:: manualDriveCallback stopped!")


def assistedDriveCallback(msg):
    # TODO algo for avoiding obstacles
    global regions
    global globalVelocity
    global velocityToSend
    global meanOfRanges

    if (msg.data == True):
        prYellow("Master:: assistedDriveCallback running!")
        prYellow(
            f"DriveAssist:: threshhold for warnings ->{assitanceThreshold}!")
        time.sleep(2)
    while (msg.data == True):
        velocityToSend = globalVelocity
        print_menu()
        prYellow("DriveAssist:: drive assist engaged!")
        if (regions['right'] <= 2*assitanceThreshold):
            print("DriveAssist:: Obstacle detected on the right side!")
            if (regions['right'] <= assitanceThreshold):
                print(f"\033[91mDistance-> {regions['right']} \033[00m")
                # velocityToSend.angular.z = -1.0 * assitanceThreshold
            else:
                print(f"\033[95mDistance-> {regions['right']} \033[00m")

        if (regions['fright'] <= 2*assitanceThreshold):
            print("DriveAssist:: Obstacle detected on the front-right side!")
            if (regions['fright'] <= assitanceThreshold):
                print(f"\033[91mDistance-> {regions['fright']} \033[00m")
                # velocityToSend.angular.z = -0.5 * assitanceThreshold

            else:
                print(f"\033[95mDistance-> {regions['fright']} \033[00m")

        if (regions['front'] <= 2*assitanceThreshold):
            print("DriveAssist:: Obstacle detected on the front side!")
            if (regions['front'] <= assitanceThreshold):
                print(f"\033[91mDistance-> {regions['front']} \033[00m")
                # velocityToSend.linear.x = -0.5 * assitanceThreshold

            else:
                print(f"\033[95mDistance-> {regions['front']} \033[00m")

        if (regions['fleft'] <= 2*assitanceThreshold):
            print("DriveAssist:: Obstacle detected on the front-left side!")
            if (regions['fleft'] <= assitanceThreshold):
                print(f"\033[91mDistance-> {regions['fleft']} \033[00m")
                # velocityToSend.angular.z = 0.5 * assitanceThreshold

            else:
                print(f"\033[95mDistance-> {regions['fleft']} \033[00m")

        if (regions['left'] <= 2*assitanceThreshold):
            print("DriveAssist:: Obstacle detected on the left side!")
            if (regions['left'] <= assitanceThreshold):
                print(f"\033[91mDistance-> {regions['left']} \033[00m")
                # velocityToSend.angular.z = 1.0 * assitanceThreshold

            else:
                print(f"\033[95mDistance-> {regions['left']} \033[00m")

        if (regions['right'] <= assitanceThreshold):
            velocityToSend.angular.z = 1.0 * \
                assitanceThreshold * (1/regions["right"])

        elif (regions['fright'] <= assitanceThreshold):
            velocityToSend.angular.z = 0.75 * \
                assitanceThreshold * (1/regions["fright"])

        elif (regions['front'] <= assitanceThreshold):
            if (regions['fright'] + regions['right'] + regions['front'] <= regions['fleft'] + regions['left'] + regions['front']):
                velocityToSend.angular.z = velocityToSend.angular.z * \
                    (-1) * (1/regions['front'])
            else:
                velocityToSend.angular.z = velocityToSend.angular.z * \
                    (1) * (1/regions['front'])

            if (regions['front'] <= 0.5):
                velocityToSend.linear.x = velocityToSend.linear.x / \
                    (1/regions['front'])

        elif (regions['fleft'] <= assitanceThreshold):
            velocityToSend.angular.z = -0.75 * \
                assitanceThreshold * (1/regions["fleft"])

        elif (regions['left'] <= assitanceThreshold):
            velocityToSend.angular.z = -1.0 * \
                assitanceThreshold * (1/regions["left"])

        pubToDrive.publish(velocityToSend)
        # prYellow("Master:: assistedDriveCallback publlished!")
    prYellow("Master:: assitedDriveCallback stopped!")

    r.sleep()

# Function for finding the closest object on 5 sectors of the scanners


def clbk_laser(msg):
    global regions
    # global meanOfRanges
    # regions = {
    #     'right':  min(min(msg.ranges[0:143]), 5),
    #     'fright': min(min(msg.ranges[144:287]), 5),
    #     'front':  min(min(msg.ranges[288:431]), 5),
    #     'fleft':  min(min(msg.ranges[432:575]), 5),
    #     'left':   min(min(msg.ranges[576:719]), 5),
    # }
    regions = {
        'right':  min(mean(msg.ranges[0:143]), 5),
        'fright': min(mean(msg.ranges[144:287]), 5),
        'front':  min(mean(msg.ranges[288:431]), 5),
        'fleft':  min(mean(msg.ranges[432:575]), 5),
        'left':   min(mean(msg.ranges[576:719]), 5),
    }
    # print("These are the regions:", regions)


def movebase_client(xUI, yUI):
    prGreen("Master::running move_base_Client")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server(rospy.Duration(timeoutROS))

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.orientation.w = 1.0
    goal.target_pose.pose.position.x = xUI
    goal.target_pose.pose.position.y = yUI

    client.send_goal(goal)
    prPurple("Waiting for the action client result")
    wait = client.wait_for_result(timeoutROS)
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        prPurple("Finished")
        return client.get_result()


# * The main funtion
if __name__ == '__main__':
    try:
        rospy.init_node('master_Robot')
        r = rospy.Rate(10)  # 10hz
        # prGreen("Master::THIS IS THE TEST AFTER SUBS")
        assitanceThreshold = rospy.get_param("/robotAsistanceParam")
        timeoutROS = rospy.get_param("/pathWaitParam")

        rospy.Subscriber("autonomousCoords", Float32MultiArray,
                         autoGoalDriveCallback)
        rospy.Subscriber("manualDriveCheck", Bool,
                         manualDriveCallback)
        rospy.Subscriber("assistedDriveCheck", Bool,
                         assistedDriveCallback)
        rospy.Subscriber("remmaped_cmd_vel", Twist,
                         makeDrivingGreatAgain)
        rospy.Subscriber("scan", LaserScan, clbk_laser)

        pubToDrive = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # result = True
        # prGreen("MASTER::THIS IS THE TEST AFTER SUBS")
        # result = movebase_client()
        # if result:
        #     rospy.loginfo("Goal execution done!")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
