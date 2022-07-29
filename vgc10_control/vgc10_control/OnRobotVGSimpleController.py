#!/usr/bin/env python3

import rclpy
from vgc10_msgs.msg import OnRobotVGOutput
import threading

def genCommand(char, command):
    """Updates the command according to the character entered by the user."""

    if char == 'g':
        command.rmca = 0x0100
        command.rvca = 255
        command.rmcb = 0x0100
        command.rvcb = 255
    if char == 'r':
        command.rmca = 0x0000
        command.rvca = 0
        command.rmcb = 0x0000
        command.rvcb = 0
    if char == 'ga':
        command.rmca = 0x0100
        command.rvca = 255
    if char == 'ra':
        command.rmca = 0x0000
        command.rvca = 0
    if char == 'gb':
        command.rmcb = 0x0100
        command.rvcb = 255
    if char == 'rb':
        command.rmcb = 0x0000
        command.rvcb = 0

    # If the command entered is a int, assign this value to r
    try:
        if int(char) == 0:
            command.rmca = 0x0000
            command.rvca = 0
            command.rmcb = 0x0000
            command.rvcb = 0
        else:
            command.rmca = 0x0100
            command.rvca = min(255, int(char))
            command.rmcb = 0x0100
            command.rvcb = min(255, int(char))
    except ValueError:
        pass

    return command


def askForCommand(command):
    """Asks the user for a command to send to the gripper."""

    currentCommand = 'Simple OnRobot VG Controller\n-----\nCurrent command:'
    currentCommand += ' rmca = ' + str(command.rmca)
    currentCommand += ', rvca = ' + str(command.rvca)
    currentCommand += ', rmcb = ' + str(command.rmcb)
    currentCommand += ', rvcb = ' + str(command.rvcb)

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
    pub = node.create_publisher(OnRobotVGOutput, 'OnRobotVGOutput', 1)
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