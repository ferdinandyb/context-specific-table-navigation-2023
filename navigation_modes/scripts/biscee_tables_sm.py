#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
import argparse
import actionlib

from smach_ros import SimpleActionState

from navigation_modes.msg import GoLocationAction,GoLocationActionFeedback, GoLocationGoal, GoLocationFeedback
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction,MoveBaseActionGoal
from robotnik_navigation_msgs.msg import DockAction,DockActionGoal,DockGoal
from biscee_tables_sm_movements import create_approach_table
from navigation_modes.msg import SetBehaviourAction,SetBehaviourActionFeedback,SetBehaviourActionGoal, SetBehaviourFeedback
from std_srvs.srv import Empty

class RecoveryBehaviour(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                outcomes = ["aborted",'retry'],
                input_keys = ['locid','GoLocationActionFeedback'],
                output_keys = ['GoLocationGoal']
                )
        self.counter = 0
        self.last_locid = ""
        self.nav = rospy.get_param('/biscee_tables/navigation_server')

    def execute(self,userdata):
        print "recovery", userdata.locid
        if userdata.locid == self.last_locid:
            self.counter += 1
        else:
            self.counter = 1
            self.last_locid = userdata.locid

        if self.counter > 3:
            self.counter = 0
            self.last_locid = ""
            return 'aborted'

        goal = GoLocationGoal()
        goal.locid = userdata.locid
        userdata.GoLocationGoal = goal
        return 'retry'

class AssessGoLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                outcomes = ["success","aborted","preempted","dock","table_manners"],
                input_keys = ['locid','GoLocationActionFeedback'],
                output_keys = ['dock_frame','robot_dock_frame','GoLocationActionFeedback']
                )
    def execute(self,userdata):
        feedback = GoLocationActionFeedback()
        if userdata.locid == 'fix/dock':
            userdata.dock_frame = "rb1_base_docking_station_contact"
            userdata.robot_dock_frame = 'rb1_base_base_docking_contact_link'
            return 'dock'
        else:
            goal = rospy.get_param('/biscee_location/' + userdata.locid)
            try:
                loctype = goal["loctype"]
            except:
                loctype = "freestanding"

            feedback.feedback = "finished"
            userdata.GoLocationActionFeedback = feedback
            print(userdata.GoLocationActionFeedback)
            return "success"

class TableManners(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                outcomes = ["aborted",'success',"preempted"],
                )
        self.param = rospy.get_param("/biscee_tables/table_manners")
        
        self.p_feedback = rospy.Publisher("/biscee_go_location/feedback",GoLocationActionFeedback,queue_size = 10)

    def execute(self,userdata):
       
        rate = rospy.Rate(10)
        t = rospy.get_time() + self.param["watch_faces_time"]
        looking_at_tray = False
       
        fb = GoLocationActionFeedback()
        fb.feedback = GoLocationFeedback("table manners")
        self.p_feedback.publish(fb)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                fb.feedback = GoLocationFeedback("table manners preempt")
                self.p_feedback.publish(fb)
                return "preempted"
            if t < rospy.get_time():
                if not looking_at_tray:
                    
                    fb.feedback = GoLocationFeedback("table manners looking at tray")
                    self.p_feedback.publish(fb)
                    looking_at_tray = True
                    self.tray()
                    t = t + self.param["watch_tray"]
                else:
                    self.center()
                    fb.feedback = GoLocationFeedback("table manners finished: looking")
                    self.p_feedback.publish(fb)
                    return "success"
            rate.sleep()
        return 'success'


class StageGoLocationGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                outcomes = ['approach_table','aborted'],
                input_keys=['GoLocationGoal'],
                output_keys=['target_pose','GoLocationActionFeedback','locid']
                )
        self.map_frame = rospy.get_param("/biscee_tables/map_frame")

    def execute(self,userdata):
        feedback = GoLocationActionFeedback()
        userdata.locid = userdata.GoLocationGoal.locid
        try:
            locations = rospy.get_param("/biscee_location")
            id_ = userdata.GoLocationGoal.locid	
            prefix, id_ = id_.split("/")
            loc = locations[prefix][id_]
            feedback.feedback = "going to {} ({})".format(loc["name"],userdata.GoLocationGoal.locid)
            userdata.GoLocationActionFeedback = feedback 
            target_pose = {
                    'header':{
                        'frame_id': self.map_frame
                        },
                    'target_pose':{
                    'pose': {
                            'x': loc['trans'][0],
                            'y': loc['trans'][1]
                        },
                    'orientation': {
                            'z': loc['rot'][0],
                            'w': loc['rot'][1]
                        }
                    }
                    }
            target_pose = PoseStamped()
            target_pose.header.stamp = rospy.Time.now()
            target_pose.header.frame_id = self.map_frame
            target_pose.pose.position.x = loc['trans'][0]
            target_pose.pose.position.y = loc['trans'][1]
            target_pose.pose.orientation.z = loc['rot'][0]
            target_pose.pose.orientation.w = loc['rot'][1] 
            userdata.target_pose = target_pose
            return 'approach_table'
        except Exception as e:
            loc = locations[prefix][id_]
            feedback.feedback =  "something went wrong aborting {}, {}".format(userdata.GoLocationGoal.locid,e)
            return 'aborted'

def main(navigation_server = "/rb1_base/move_base"):
    rospy.init_node("biscee_tables_sm")

    rospy.loginfo("%f",rospy.get_time())

    sm = smach.StateMachine(
            outcomes=["success","aborted","preempted"],
            input_keys = ["GoLocationGoal"],
            )
    ApproachTableCC = create_approach_table()
    with sm:

        smach.StateMachine.add('StageGoLocationGoal',
                StageGoLocationGoal(),
                transitions={'approach_table':'ApproachTableCC'},
                )

        smach.StateMachine.add('ApproachTableCC',ApproachTableCC,
                transitions = {
                    'golocationsuccess': 'AssessGoLocation',
                    'approachsuccess': 'AssessGoLocation',
                    'recoverytab': 'RecoveryBehaviour',
                    'recoveryloc': 'RecoveryBehaviour',
                    'preempted': 'preempted'
                    } 
                )
        smach.StateMachine.add('AssessGoLocation',
                AssessGoLocation(),
                transitions={
                    'success':'success',
                    'dock':'Dock',
                    "table_manners":"TableManners"
                    }
                )

        smach.StateMachine.add('Dock',
                    SimpleActionState('/rb1_base/docker',
                                        DockAction,
                                        goal_slots = ['dock_frame','robot_dock_frame']
                                        ),
                        transitions = {'succeeded':'success'}
                )


        smach.StateMachine.add('RecoveryBehaviour',
                RecoveryBehaviour(),
                transitions={'aborted':'aborted','retry':'StageGoLocationGoal'}
                )
        smach.StateMachine.add('TableManners',
                TableManners(),
                transitions={
                    'aborted':'aborted',
                    'success':'success',
                    "preempted":"preempted"
                    }
                )

    asw = smach_ros.ActionServerWrapper(
       'biscee_go_location',
       GoLocationAction,
       wrapped_container = sm,
       succeeded_outcomes = ["success"],
       aborted_outcomes = ['aborted'],
       preempted_outcomes = ['preempted'],
       goal_key = 'GoLocationGoal',
       feedback_key = "GoLocationActionFeedback"
    )

    sis = smach_ros.IntrospectionServer('server_name', sm, '/biscee_tables_sm')
    sis.start()

    asw.run_server()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-ns","--navserver",type=str, default = "/rb1_base/move_base")
    args,unknownargs = parser.parse_known_args()
    main(navigation_server = args.navserver)
