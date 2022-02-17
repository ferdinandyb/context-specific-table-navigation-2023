#!/usr/bin/env python

import rospy
import smach
import numpy as np
import time
import smach_ros
from smach_ros import SimpleActionState,ActionServerWrapper

from navigation_modes.msg import GoLocationActionGoal,GoLocationAction
from navigation_modes.msg import SetBehaviourAction,SetBehaviourActionFeedback,SetBehaviourActionGoal 

class ChooseLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                outcomes = ["GoLocation","success","preempted"],
                output_keys = ['locid','feedback'],
                input_keys = ['behaviour'])
        self.previous_goal = ''
        self.goal_id = 0
        self.table_test_goals = [
            "fix/base",
            "mutable/1",
            "fix/base",
            "mutable/2",
            "fix/base",
            "mutable/3",
            "fix/base",
            "mutable/4",
            "fix/base",
            "sleep",
            "fix/base",
            "mutable/4",
            "fix/base",
            "mutable/3",
            "fix/base",
            "mutable/2",
            "fix/base",
            "mutable/1",
            "fix/base",
            "sleep",
            "fix/base",
            "mutable/1",
            "mutable/2",
            "mutable/3",
            "mutable/4",
            "fix/base",
            "sleep",
            "fix/base",
            "mutable/4",
            "mutable/3",
            "mutable/2",
            "mutable/1",
            "fix/base",
            "sleep",
            "fix/base",
            "mutable/4",
            "mutable/1",
            "mutable/3",
            "mutable/2",
            "fix/base",
            "sleep",
            "fix/base",
            "mutable/2",
            "mutable/3",
            "mutable/1",
            "mutable/4",
            "fix/base",
            "sleep",
            "fix/base",
            "mutable/3",
            "mutable/2",
            "mutable/3",
            "mutable/2",
            "fix/base",
            "sleep",
            "fix/base",
            "mutable/2",
            "mutable/3",
            "mutable/2",
            "mutable/3",
            "fix/base",
            "sleep",
            "fix/base",
            "mutable/1",
            "mutable/4",
            "mutable/1",
            "mutable/4",
            "fix/base",
            "sleep",
            "fix/base",
            "mutable/4",
            "mutable/1",
            "mutable/4",
            "mutable/1",
            "fix/base",
            "sleep",
            "fix/base",
            "mutable/1",
            "mutable/2",
            "mutable/1",
            "mutable/3",
            "mutable/1",
            "mutable/4",
            "fix/base",
            "sleep",
            "fix/base",
            "mutable/4",
            "mutable/3",
            "mutable/4",
            "mutable/2",
            "mutable/4",
            "mutable/1",
            "fix/base",
            "fix/base"
        ]

    def execute(self,userdata):
        if self.preempt_requested():
            self.service_preempt()
            return "preempted"
        userdata.locid = "fix/base"
        locations = rospy.get_param("/biscee_location")["mutable"]
        feedback = SetBehaviourActionFeedback()
        print(userdata.behaviour,self.previous_goal)
        if userdata.behaviour.behaviour == 'random':
            rospy.set_param("/biscee_behaviour/patrol","random")
            choice = np.random.choice(locations.keys())
            userdata.locid = 'mutable/' + choice
            self.previous_goal = 'mutable/' + choice
            feedback.feedback = "going to {}".format(self.previous_goal)
            userdata.feedback = feedback
            return "GoLocation"
        elif userdata.behaviour.behaviour == "base_table":
            rospy.set_param("/biscee_behaviour/patrol","base_table")
            time.sleep(2)
            if self.previous_goal == "fix/base":
                choice = np.random.choice(locations.keys())
                userdata.locid = 'mutable/' + choice
                self.previous_goal = 'mutable/' + choice
            else:
                userdata.locid = "fix/base"
                self.previous_goal = "fix/base"
            feedback.feedback = "going to {}".format(self.previous_goal)
            userdata.feedback = feedback
            return "GoLocation"
        elif userdata.behaviour.behaviour == "table_test":
            if not rospy.get_param("/biscee_behaviour/patrol") == "table_test":
                self.goal_id = 0
            rospy.set_param("/biscee_behaviour/patrol","table_test")
            if self.table_test_goals[self.goal_id] == "sleep":
                time.sleep(5)
                self.goal_id += 1
            userdata.locid = self.table_test_goals[self.goal_id]
            feedback.feedback = "going to {}".format(self.table_test_goals[self.goal_id])
            userdata.feedback = feedback
            self.goal_id += 1
            if self.goal_id == len(self.table_test_goals):
                return "success"
            else:
                return "GoLocation"
        else:
            rospy.set_param("/biscee_behaviour/patrol","none")
            feedback.feedback = "stopping"
            userdata.feedback = feedback
            return 'success'


if __name__ == "__main__":
    rospy.init_node("biscee_patrol_sm")
    rospy.set_param("/biscee_behaviour/patrol","none")
    sm = smach.StateMachine(
            outcomes = ["success","aborted","preempted"],
            input_keys = ['behaviour'],
            output_keys = ['feedback']
            )
    
    with sm:
        smach.StateMachine.add("ChooseLocation",
                ChooseLocation(),
                transitions = {'GoLocation':'GoLocation',
                    "success":"success"}
                )
    
        smach.StateMachine.add('GoLocation',
                SimpleActionState('/biscee_go_location',
                GoLocationAction,
                goal_slots = ['locid']
                ),
                transitions = {'succeeded':'ChooseLocation',
                                'aborted': 'ChooseLocation',
                                'preempted':'preempted'
                        }
                )
    
    asw = ActionServerWrapper(
        'biscee_patrol_behaviour',
        SetBehaviourAction,
        wrapped_container = sm,
        succeeded_outcomes = ["success"],
        aborted_outcomes = ['aborted'],
        preempted_outcomes = ['preempted'],
        goal_key = "behaviour",
        feedback_key = "feedback"
        )

    sis = smach_ros.IntrospectionServer('server_name', sm, '/biscee_patrol_sm')
    sis.start()

    asw.run_server()
    rospy.spin()
