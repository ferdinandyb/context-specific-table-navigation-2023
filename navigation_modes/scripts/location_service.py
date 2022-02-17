#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import String,Int8
from std_srvs.srv import Trigger
from navigation_modes.srv import LocationID
from geometry_msgs.msg import PoseWithCovarianceStamped 
import yaml


class LocationServer():
    def __init__(self,path):
        self.path = path
        self.data = {} 
        self.highest_id = 0

        self.new_loc = rospy.Service("biscee_locmanager_newloc",LocationID,self.newloc_cb)
        self.delete_loc = rospy.Service("biscee_locmanager_deleteloc",LocationID,self.deleteloc_cb)
        self.update_loc_data = rospy.Service("biscee_locmanager_updatelocdata",LocationID,self.updatelocdata_cb)
        self.update_loc_pos = rospy.Service("biscee_locmanager_updatelocpos",LocationID,self.updatelocpos_cb)

        self.init_pose = rospy.Service('biscee_init_pose',LocationID,self.init_pose)
        self.init_pose_pub = rospy.Publisher('/rb1_base/initialpose',PoseWithCovarianceStamped,queue_size = 10)
        self.load_data()

    def load_data(self):
        try:
            rospy.delete_param('/biscee_location')
        except KeyError:
            pass
        try:
            self.data = yaml.load(open(self.path))
        except:
            self.data = {'mutable':{},
                    'fix':{'base': {'name': 'base', 'loctype': 'freestanding'},
                            'dock': {'name': 'dock', 'loctype': 'freestanding'}
                            }
                        }
        rospy.set_param('biscee_location',self.data)
        self.set_highest_id()
      
    def set_highest_id(self):
        try:
            self.highest_id = max([int(i) for i in self.data['mutable'].keys()])
        except ValueError:
            self.highest_id = 0


    def save_data(self):
        yaml.dump(self.data,open(self.path,"w"))
        rospy.set_param('biscee_location',self.data)

    def get_current_location(self):
        listener = tf.TransformListener()
        try:
            listener.waitForTransform("rb1_base_map","rb1_base_base_link",rospy.Time(0),rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform("/rb1_base_map", "/rb1_base_base_link", rospy.Time(0))
            loc = {
                'trans': [trans[0],trans[1]],
                'rot': [rot[2],rot[3]]
                    }
            return loc,True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            loc = {
                'trans': [0,0],
                'rot': [0,0]
                    }
            return loc,False

    def init_pose(self,req):
        try:
            pose_stamped = PoseWithCovarianceStamped()
            pose_stamped.header.frame_id = "rb1_base_map"
            pose_stamped.header.stamp = rospy.Time.now()
            coord = rospy.get_param("/biscee_location/" + req.type + "/" + req.id)
            pose_stamped.pose.pose.position.x = coord["trans"][0]
            pose_stamped.pose.pose.position.y = coord["trans"][1]
            pose_stamped.pose.pose.position.z = 0
            pose_stamped.pose.pose.orientation.x = 0
            pose_stamped.pose.pose.orientation.y = 0
            pose_stamped.pose.pose.orientation.z = coord["rot"][0]
            pose_stamped.pose.pose.orientation.w = coord["rot"][1]
            self.init_pose_pub.publish(pose_stamped)
            return req.id
        except  Exception as e:
            print e
            return -1

    def newloc_cb(self,req):
        if not req.type == "mutable":
            return str(-1)
        loc,success = self.get_current_location()
        if not success:
            return str(-1)
        newid = self.highest_id + 1
        self.highest_id += 1
        loc["loctype"] = req.loctype
        loc["name"] = str(newid)
        self.data['mutable'][str(newid)] = loc
        self.save_data()
        return str(newid)

    def deleteloc_cb(self,req):
        if not req.type == "mutable":
            return str(-1)
        try:
            self.data["mutable"].pop(req.id,None)
            rospy.delete_param("/biscee_location/mutable/"+ req.id)
            self.save_data()
            self.set_highest_id()
            return req.id
        except KeyError:
            return str(-1)

    def updatelocdata_cb(self,req):
        try:
            self.data[req.type][req.id]["loctype"] = req.loctype
            self.data[req.type][req.id]["name"] = req.name
            self.save_data()
            return req.id
        except KeyError:
            return str(-1)

    def updatelocpos_cb(self,req):
        loc,success = self.get_current_location()
        if not success:
            return str(-1)
        try:
            print(self.data)
            self.data[req.type][req.id]["trans"] = loc["trans"]
            self.data[req.type][req.id]["rot"] = loc["rot"]
            self.save_data()
            return req.id
        except KeyError:
            return str(-1)

if __name__ == '__main__':
    import argparse
    import os
    parser = argparse.ArgumentParser()
    parser.add_argument("-ph", "--path", type=str,
                        help="path to the save_file")
    args, unknownargs = parser.parse_known_args()
    path_saveFile = os.path.join(os.path.expanduser('~'), 'save_location.yaml')
    path = args.path if args.path else path_saveFile
    print path


    rospy.init_node("biscee_save_location_service")

    locserv = LocationServer(path)

    # s_to_file = rospy.Service('biscee_save_location_to_file',Trigger,save_location_to_file)
    # s_save_loc = rospy.Service('biscee_save_current_location',LocationID,save_current_location)
    # s_save_loc_fix = rospy.Service('biscee_save_current_location_fix',LocationID,save_fix)
    # s_delete_loc = rospy.Service('biscee_delete_location',LocationID,remove_current_location)
    rospy.spin()
