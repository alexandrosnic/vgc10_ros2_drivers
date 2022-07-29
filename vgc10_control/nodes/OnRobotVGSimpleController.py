#!/usr/bin/env python3

import rclpy
from vgc10_msgs.msg import OnRobotVGOutput
import threading

def genCommand(char, command):
    """Updates the command according to the character entered by the user."""

    if char == 'g':
        command.rMCA = 0x0100
        command.rVCA = 255
        command.rMCB = 0x0100
        command.rVCB = 255
    if char == 'r':
        command.rMCA = 0x0000
        command.rVCA = 0
        command.rMCB = 0x0000
        command.rVCB = 0
    if char == 'ga':
        command.rMCA = 0x0100
        command.rVCA = 255
    if char == 'ra':
        command.rMCA = 0x0000
        command.rVCA = 0
    if char == 'gb':
        command.rMCB = 0x0100
        command.rVCB = 255
    if char == 'rb':
        command.rMCB = 0x0000
        command.rVCB = 0

    # If the command entered is a int, assign this value to r
    try:
        if int(char) == 0:
            command.rMCA = 0x0000
            command.rVCA = 0
            command.rMCB = 0x0000
            command.rVCB = 0
        else:
            command.rMCA = 0x0100
            command.rVCA = min(255, int(char))
            command.rMCB = 0x0100
            command.rVCB = min(255, int(char))
    except ValueError:
        pass

    return command


def askForCommand(command):
    """Asks the user for a command to send to the gripper."""

    currentCommand = 'Simple OnRobot VG Controller\n-----\nCurrent command:'
    currentCommand += ' rMCA = ' + str(command.rMCA)
    currentCommand += ', rVCA = ' + str(command.rVCA)
    currentCommand += ', rMCB = ' + str(command.rMCB)
    currentCommand += ', rVCB = ' + str(command.rVCB)

    # rospy.loginfo(currentCommand)
    # node.get_logger().info(currentCommand)

    strAskForCommand = '-----\nAvailable commands\n\n'
    strAskForCommand += 'g: Turn on all channels\n'
    strAskForCommand += 'r: Turn off all channels\n'
    strAskForCommand += 'ga: Turn on channel A\n'
    strAskForCommand += 'ra: Turn off channel A\n'
    strAskForCommand += 'gb: Turn on channel B\n'
    strAskForCommand += 'rb: Turn off channel B\n'
    strAskForCommand += '(0 - 255): Set vacuum power for all channels\n'

    strAskForCommand += '-->'

    return input(strAskForCommand)


def publisher(args=None):
    """Main loop which requests new commands and
       publish them on the OnRobotVGOutput topic.
    """

    rclpy.init(args=args)
    node = rclpy.create_node('OnRobotVGSimpleController')
    pub = node.create_publisher(OnRobotVGOutput, 'OnRobotVGOutput', queue_size=1)
    command = OnRobotVGOutput()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(0.1)
    try:
        while rclpy.ok():
            command = genCommand(askForCommand(command), command)
            pub.publish(command)
            rate.sleep()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    publisher()