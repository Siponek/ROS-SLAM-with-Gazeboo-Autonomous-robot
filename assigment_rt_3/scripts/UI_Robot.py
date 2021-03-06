#!/usr/bin/env python

import rospy
import std_msgs.msg
# *for string, bool, empty
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import time
from move_base_msgs import msg
from actionlib_msgs.msg import GoalID

# pubGoalPoint = None
pubFromUIFLOAT = None
pubSignalManual = None
pubSignalAssisted = None
pubSignalCancel = None

menu_options = {
    1: 'Autonomously reach a x,y coordinate inserted by the user',
    2: 'Drive the robot with the keyboard',
    3: 'Drive the robot with assistance to avoid collisions',
    4: 'Exit',
    5: "IT'S TIME TO STOP",
    6: "Cancel goal",
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
    print("\033[96m")

    for key in menu_options.keys():
        print(key, '--', menu_options[key])
    print("\033[00m")


def option1():
    xInput = float(input("Please type in an X coord"))
    yInput = float(input("Please type in an Y coord"))
    arrayToSend = std_msgs.msg.Float32MultiArray()
    arrayToSend.data = [xInput, yInput]
    print('Sending coordinates to master_Robot node -> move_base')

    pubFromUIFLOAT.publish(arrayToSend)


def option2():
    print('Drive mode: Manual')
    signalToSendManual = std_msgs.msg.Bool()
    signalToSendManual.data = True

    signalToSendAssisted = std_msgs.msg.Bool()
    signalToSendAssisted.data = False

    print('Setting up manual drive mode...')
    pubSignalManual.publish(signalToSendManual)


def option3():
    print('Drive mode: Assisted')
    signalToSendAssisted = std_msgs.msg.Bool()
    signalToSendAssisted.data = True

    signalToSendManual = std_msgs.msg.Bool()
    signalToSendManual.data = False

    print('Setting up assisted drive mode...')
    pubSignalAssisted.publish(signalToSendAssisted)


def option5():
    print('Reseting drive modes...')
    signalToSendAssisted = std_msgs.msg.Bool()
    signalToSendAssisted.data = False

    signalToSendManual = std_msgs.msg.Bool()
    signalToSendManual.data = False

    pubSignalManual.publish(signalToSendManual)
    pubSignalAssisted.publish(signalToSendAssisted)


def option6():
    print('Canceling...')
    signalCancel = GoalID()

    pubSignalCancel.publish(signalCancel)


if __name__ == '__main__':
    rospy.init_node('UI_Robot')
    pubFromUIFLOAT = rospy.Publisher(
        "autonomousCoords", std_msgs.msg.Float32MultiArray, queue_size=1)
    pubSignalManual = rospy.Publisher(
        "manualDriveCheck", std_msgs.msg.Bool, queue_size=1)
    pubSignalAssisted = rospy.Publisher(
        "assistedDriveCheck", std_msgs.msg.Bool, queue_size=1)

    pubSignalCancel = rospy.Publisher(
        "move_base/cancel", GoalID, queue_size=1)
    print_menu()
    while(True):
        option = None
        try:
            option = int(input('Enter your choice: '))
        except:
            print('Wrong input. Please enter a number ...')
        # Check what choice was entered and act accordingly
        if option == 1:
            option1()

        elif option == 2:
            option2()

        elif option == 3:
            option3()

        elif option == 4:
            print('Thanks message before exiting')
            # TODO rospy.shutdown() on other nodes as well
            # rospy.signal_shutdown("User choice (4)")
            # rospy.spin()
            exit()

        elif option == 5:
            option5()

        elif option == 6:
            option6()

        else:
            print('Invalid option. Please enter a number between 1 and 4.')
