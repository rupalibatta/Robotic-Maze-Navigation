#!/usr/bin/env python3
#
#   ros.py
#
#   Set up a very simple ROS node to listen for goal/explore commands
#   and publish the current pose (position and heading).
#
#   Node:       /PINAME         (this will use your Pi's name)
#
#   Publish:    ~/pose                  geometry_msgs/Pose
#   Subscribe:  ~/goal                  geometry_msgs/Point
#   Subscribe:  ~/explore               std_msgs/Empty
#
import ctypes
import os
import rclpy
import socket
import time
import threading

from math import pi, sin, cos

from rclpy.node                 import Node
from rclpy.time                 import Time, Duration
from geometry_msgs.msg          import Point, Pose
from std_msgs.msg               import Empty


#
#   Simple ROS Node Class
#
class ROSNode(Node):
    # Initialization.
    def __init__(self, name, shared):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Save the shared data object.
        self.shared = shared

        # Create the publisher for the pose information.
        self.pub = self.create_publisher(Pose, '~/pose', 10)

        # Then create subscribers for goal and explore commands.
        self.create_subscription(Point, '~/goal',    self.cb_goal,    10)
        self.create_subscription(Empty, '~/explore', self.cb_explore, 10)

        # Finally create a timer to drive the node.
        self.timer = self.create_timer(1.0, self.cb_timer)

        # Report and return.
        self.get_logger().info("ROS Node '%s' running" % (name))

    # Shutdown.
    def shutdown(self):
        # Destroy the timer and shut down the node.
        self.destroy_timer(self.timer)
        self.destroy_node()


    # Timer callback.  Send the robot's current pose.
    def cb_timer(self):
        # Grab the current x/y and heading from the shared data.
        if self.shared.acquire():
            posx  = self.shared.robotx
            posy  = self.shared.roboty
            head  = self.shared.robotheading
            self.shared.release()

        # Convert the heading into an angle (in radians).
        theta = pi/4 * float(head+2)

        # Populate the ROS message with the data and send.  The
        # orientation is encoded as a quaternion.
        msg = Pose()
        msg.position.x    = float(posx)
        msg.position.y    = float(posy)
        msg.orientation.z = sin(theta/2)
        msg.orientation.w = cos(theta/2)
        self.pub.publish(msg)


    # Goal command callback.
    def cb_goal(self, msg):
        # Extract the goal coordinates from the message.
        xgoal = msg.x
        ygoal = msg.y

        # Report.
        self.get_logger().info("Received goal command (%d,%d)" % (xgoal,ygoal))

        # Inject the goal command into your robot thread, just as if
        # you had typed the goal command in the UI thread.

        if self.shared.acquire():
            self.shared.command = 'goal'
            # store as strings to match the UI parsing
            self.shared.params = [str(int(xgoal)), str(int(ygoal))]
            self.shared.release()

    # Explore command callback.
    def cb_explore(self, msg):
        # Report.
        self.get_logger().info("Received explore command")

        # Inject the explore command into your robot thread, just as
        # if you had typed the explore command in the UI thread.

        # Inject as if typed in UI
        if self.shared.acquire():
            self.shared.command = 'explore'
            self.shared.params  = []
            self.shared.release()

#
#   Main ROS Thread Code
#
def runros(shared):
    # Setup network access for ROS on domain #1.
    os.environ['ROS_LOCALHOST_ONLY']='0'
    os.environ['ROS_DOMAIN_ID']='1'

    # Initialize ROS.
    rclpy.init()

    # Instantiate a simple ROS node, named after the host name, and
    # passing the shared data object.
    node = ROSNode(socket.gethostname(), shared)

    # Spin the node until interrupted.
    try:
        rclpy.spin(node)
    except BaseException as ex:
        print("Ending Run-ROS due to exception: %s" % repr(ex))

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()



######################################################################
#
#   Testing Code
#
#   This executes if run stand-alone.  Similar functionality should be
#   in your existing code.
#
if __name__ == "__main__":

    ### Dummy shared data object and run UI function for testing:

    # Define the shared data here for testing.
    class SharedData:
        # Initialize the data.
        def __init__(self):
            # Create the lock.
            self.lock = threading.Lock()

            # Initialize the data.  The following are example data:
            self.robotx = 24
            self.roboty = 35
            self.robotheading = 3

        # Method to acquire/gain access to the shared data.
        def acquire(self):
            return self.lock.acquire()

        # Method to release/relinquish access to the shared data.
        def release(self):
            self.lock.release()

    # Define a dummy robot function for testing.
    def runrobot(shared):
        # Just sleep a lot.  You would have the robot code here.
        try:
            while True:
                time.sleep(1)

        # If signaled, break out of the infinite loop and return.
        except BaseException as ex:
            print("Ending Run-Robot due to exception: %s" % repr(ex))


    ### Dummy code for testing:

    # Instantiate the shared data.
    shared = SharedData()

    # Run the ROS thread - notice the argument.
    rosthread = threading.Thread(
        name="ROSThread", target=runros, args=(shared,))
    rosthread.start()

    # Run the ROBOT thread.
    runrobot(shared)

    # Send an exception (Keyboard Interrupt) to the ROS thread
    # to finish. Then wait to join.
    print("Interrupting ROS thread...")
    ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_long(rosthread.ident),
        ctypes.py_object(KeyboardInterrupt))
    rosthread.join()

    # Nothing to shutdown
    print("Exiting")
