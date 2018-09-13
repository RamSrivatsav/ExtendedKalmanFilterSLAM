"""
Created on Thu Sept 1 10:53:08 2016
@author: admin-u5941570

"""

Helperscripts are meant to help you generate the "Tarjectory and Points" file after you have created your "Measurement" file running the rosbags and writing each measurement on a different line.

HOW TO:
    1- Run the rosbag(s) --> $rosbag play --pause sample#.bag
    2- Write odometry measurement and landmark measurement each on a different line in your "Measurement" file.
    3- Use the helper scripts along with your "Measurement" file to generate robot absolute poses and landmark 
    absolute  positions.
    4- Plot the robot trajectory using robot absolute poses and landmark positions in a single plot.

