#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Range
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header
from geometry_msgs.msg import Point

'''
Convert ultrasounds to PointCloud.

Also merge front_bottomlow and front_bottomhigh into a single front_bottom unit
'''

ultras = [
        "front_top",
        "left_45",
        "right_45",
        "left_90",
        "right_90"
        ]
pubs = [rospy.Publisher('/biscee_ultra_cloud_' + u, PointCloud, queue_size=10) for u in ultras]

class FrontBottom:
    def __init__(self):
        self.pub_cloud = rospy.Publisher('/biscee_ultra_cloud_front_bottom', PointCloud, queue_size = 10)
        self.pub_range = rospy.Publisher('/ultrasound_distance_front_bottom', Range, queue_size = 10)
        self.range = [100,100]
        rospy.Subscriber('/ultrasound_distance_front_bottomlow',Range,self.callback,0)
        rospy.Subscriber('/ultrasound_distance_front_bottomhigh',Range,self.callback,1)

    def callback(self,data,arg):
        self.range[arg] = data.range
        if 0 in self.range:
            dist = np.max(self.range)
        else:
            dist = np.min(self.range)

        point = PointCloud()
        point.header.stamp = data.header.stamp
        point.header.frame_id = "rb1_base_front_bottom_ultrasound_base_link"
        point.points.append(Point())
        point.points[0].x = dist
        point.points[0].y = 0
        point.points[0].z = 0
        self.pub_cloud.publish(point)
        
        r = Range()
        r.header.stamp = data.header.stamp
        r.header.frame_id = "rb1_base_front_bottom_ultrasound_base_link"
        r.range = dist
        r.max_range = 6.47
        self.pub_range.publish(r)

def callback(data,arg):
    global pubs
    point = PointCloud()
    point.header = data.header
    point.points.append(Point())
    point.points[0].x = data.range
    point.points[0].y = 0
    point.points[0].z = 0
    pubs[ultras.index(arg)].publish(point)


    
def main():

    rospy.init_node('biscee_ultrasound_translater', anonymous=True)
    subs = [rospy.Subscriber("/ultrasound_distance_" + u, Range, callback,u) for u in ultras]
    fb = FrontBottom()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
