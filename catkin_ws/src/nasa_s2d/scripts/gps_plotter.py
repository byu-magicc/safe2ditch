#!/usr/bin/env python
import pyqtgraph as pg
import time, tf
import numpy as np
import utm
import thread

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

# How many targets are there?
targets = 1

# initialize variables
plot_init = True # center square plot over initial data
tracker_state_list = [] # list of tracker's state [x,y,z,roll,pitch,yaw]
target_state_list = [] # list of target's state [x,y,z,roll,pitch,yaw]

# initialize plot
app = pg.QtGui.QApplication([])
pw = pg.plot()
pw.setWindowTitle('Tracker/Target Locations')
pw.setLabel('left', 'North/South') # set axis labels
pw.setLabel('bottom', 'East/West')
pw.showGrid(x=True,y=True)
pw.resize(800,800)
pw.addLegend()

def setup_legend():
    pass

# tracker path and orientation objects
tracker_path = pw.plot(name='tracker_path')
tracker_path.setPen(width=2, color=(255,0,0)) # set pen color
tracker_ori = pw.plot()
tracker_ori.setPen(width=3, color=(0,255,0)) # set pen color

# target path and orientation objects
for i in xrange(targets):
    target_path = pw.plot(name='target_path')
    target_path.setPen(width=2, color=(0,0,255)) # set pen color
    target_ori = pw.plot()
    target_ori.setPen(width=3, color=(0,255,0)) # set pen color

# my arrow in body coordinates to show orientation (UTM)
my_arrow = np.array(((0,1,0.7,1,0.7),(0,0,-0.2,0,0.2),(0,0,0,0,0)))

# this function stores the tracker's position and orientation in UTM coordinates
def uav_callback(msg):
    # orientation in quaternion form
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)

    # Use ROS tf to convert to Euler angles from quaternion
    euler = tf.transformations.euler_from_quaternion(quaternion)

    # save pose
    tracker_state_list.append(np.array((msg.pose.pose.position.x, \
                                        msg.pose.pose.position.y, \
                                        msg.pose.pose.position.z, \
                                        euler[0],                 \
                                        euler[1],                 \
                                        euler[2])))

# this function stores the target's position and orientation in UTM coordinates
def target_callback(target, msg):
    # Android GPS Beacon doesn't give orientation :(
    euler = [0, 0, 0]

    # Convert from lat/lon to UTM (https://pypi.python.org/pypi/utm)
    u = utm.from_latlon(msg.latitude, msg.longitude)

    # save pose
    target_state_list.append(np.array((u[0], \
                                        u[1], \
                                        msg.altitude, \
                                        euler[0],                 \
                                        euler[1],                 \
                                        euler[2])))

# this function plots the position and orientation of the tracker in the UTM 'inertial' frame
def draw_tracker():
    # necessary global variables
    global plot_init

    # center the plot over initial data
    if plot_init:
        pw.setXRange(tracker_state_list[-1][0]-100,tracker_state_list[-1][0]+100)
        pw.setYRange(tracker_state_list[-1][1]-100,tracker_state_list[-1][1]+100)
        plot_init = False

    # draw the path
    x_values = np.vstack(tracker_state_list)[:,0]
    y_values = np.vstack(tracker_state_list)[:,1]
    if x_values.size == y_values.size:
        tracker_path.setData(x=x_values, y=y_values)

    # build rotation matrix
    phi = tracker_state_list[-1][3]
    theta = tracker_state_list[-1][4]
    psi = tracker_state_list[-1][5]
    R_v2_to_b = np.array(((1.,  0.         , 0.         ),
                          (0.,  np.cos(phi), np.sin(phi)),
                          (0., -np.sin(phi), np.cos(phi))))
    R_v1_to_v2 = np.array(((np.cos(theta), 0., -np.sin(theta)),
                           (0.           , 1.,  0.           ),
                           (np.sin(theta), 0.,  np.cos(theta))))
    R_v_to_v1 = np.array((( np.cos(psi), np.sin(psi), 0.),
                          (-np.sin(psi), np.cos(psi), 0.),
                          ( 0.         , 0.         , 1.)))
    R_v_to_b = R_v2_to_b.dot(R_v1_to_v2.dot(R_v_to_v1))

    # translation vector
    p_trans = np.atleast_2d(tracker_state_list[-1][0:3]).transpose()

    # compute new orientation arrow in 3-D
    p_new = R_v_to_b.transpose().dot(my_arrow) + p_trans

    # draw orientation
    tracker_ori.setData(x=p_new[0,:], y=p_new[1,:])

# this function plots the position and orientation of the target in the UTM 'inertial' frame
def draw_target():
    # draw the path
    x_values = np.vstack(target_state_list)[:,0]
    y_values = np.vstack(target_state_list)[:,1]
    if x_values.size == y_values.size:
        target_path.setData(x=x_values, y=y_values)

def plot_data():
    # set the drawn data
    if tracker_state_list: # make sure list is not empty
        draw_tracker()
    if target_state_list:
        draw_target()

    # update the gui
    app.processEvents()

def main():
    # initialize node
    rospy.init_node('gps_plotter', anonymous=True)

    # The UAV subscriber expects the `/mavros/global_position/local` message from MAVROS
    rospy.Subscriber('uav', Odometry, uav_callback)

    # For every target, create a listener
    for i in xrange(targets):
        rospy.Subscriber('target{}'.format(i), NavSatFix, lambda msg: target_callback(i, msg))

    # Add a legend for UAV and targets
    setup_legend()

    # listen for messages and plot
    while not rospy.is_shutdown():
        try:
            # plot the local positions of each vehicle
            plot_data()

            # let it rest a bit
            time.sleep(0.01)
        except rospy.ROSInterruptException:
            print "exiting...."
            return

if __name__ == '__main__':
    main()