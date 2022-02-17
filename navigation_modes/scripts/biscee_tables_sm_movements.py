#!/usr/bin/env python

import rospy
import smach
import tf
import time
import actionlib

from smach_ros import SimpleActionState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Range,LaserScan
from geometry_msgs.msg import Twist,Quaternion
from actionlib_msgs.msg import GoalID
from nav_msgs.msg import Path
import numpy as np
from scipy.spatial.transform import Rotation

from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction,MoveBaseActionGoal
from navigation_modes.msg import SetBehaviourAction,SetBehaviourActionFeedback,SetBehaviourActionGoal, SetBehaviourFeedback
from navigation_modes.msg import GoLocationAction,GoLocationActionFeedback, GoLocationGoal, GoLocationFeedback

class ApproachTable(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes = ["success","preempted","aborted"],
            input_keys = ['locid']
            )
        self.time = rospy.get_time() 
        self.stalled = False
        self.sovereignty_allowed = False
        self.sovereign = False
        self.last_sov_time = 0
        self.sovereignty_dist = rospy.get_param("/biscee_tables/sovereignty_dist")
        self.control = False
        self.time_control = rospy.get_time()
        self.recovery_time = 0
        self.is_recovery = False
        self.nav_vel = 0
        self.nav_vel_at_change = 0
        self.distance_from_goal = 100
        self.safety_distance = rospy.get_param("/biscee_tables/safety_distance")
        self.control_tau =  rospy.get_param("/biscee_tables/control_tau")
        self.recovery_w  =  rospy.get_param("/biscee_tables/recovery_w")
        self.recovery_cone = rospy.get_param("/biscee_tables/recovery_cone")*2*np.pi/360
        self.p_feedback = rospy.Publisher("/biscee_go_location/feedback",GoLocationActionFeedback,queue_size = 10)
        self.path_point = None
        self.locid = ''
        self.tf_listener = tf.TransformListener()

        self.cmd_vel_pub = rospy.Publisher(
                                rospy.get_param('/biscee_tables/cmd_vel_topic'),
                                Twist,
                                queue_size=1)

        self.map_frame = rospy.get_param("/biscee_tables/map_frame")
        self.base_link_frame = rospy.get_param("/biscee_tables/base_link_frame")
        self.ultrasound_points = [[100,100,0] for x in range(5)]
        self.ultrasound_distances = [100 for x in range(5)]
        self.ultras = rospy.get_param("/biscee_tables/ultra_topics")

        self.laser_distances = [100] #

        for i,u in enumerate(self.ultras):
            rospy.Subscriber(u,Range,self.ultracb,i)
        rospy.Subscriber(rospy.get_param('/biscee_tables/laser_topic'),LaserScan,self.lasercb)
        rospy.Subscriber(rospy.get_param('/biscee_tables/navigation_server') + "/cmd_vel",Twist,self.navcmdcb)
        rospy.Subscriber(rospy.get_param('/biscee_tables/path_topic'),Path,self.pathcb)

    def pathcb(self,data):
        self.path_point = data.poses[-1].pose

    def navcmdcb(self,data):
        self.nav_vel = data.linear.x
        self.nav_w = data.angular.z

    def angle_from_quat(self,quat):
        if type(quat) is Quaternion:
            quat = [quat.x,quat.y,quat.z,quat.w]
        return euler_from_quaternion(quat)[-1]

    def get_distance(self,trans1,trans2):
        return np.sqrt((trans1[0]-trans2[0])**2 + (trans1[1] - trans2[1])**2)

    def ultracb(self,data,ind):
        try:
            trans,rot = self.tf_listener.lookupTransform(self.map_frame,data.header.frame_id,rospy.Time(0))
            r = Rotation.from_quat(rot)
            self.ultrasound_points[ind] = r.apply(np.array([data.range,0,0]))+np.array(trans)
            self.ultrasound_distances[ind] = data.range
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def lasercb(self,data):
        self.laser_distances = data.ranges

    def laser_is_close(self):
        # sides need to be checked
        if np.min(self.laser_distances) < 0.10:
            return True
        else:
            return False

    def set_stalled(self):
        if np.min(self.ultrasound_distances) < self.safety_distance or self.laser_is_close():
            if not self.stalled:
                self.stalled = True
                self.time = rospy.get_time()
            # when stuff is closed but we already stalled
            # self.time will remain the same
        else:
            self.stalled = False
            self.time = rospy.get_time()



    def set_sovereignty(self):
        # evidently, this could get fancier a bit
        sovtimelimit = 2
        if self.sovereignty_allowed:
            if rospy.get_param('/biscee_tables/is_ultrasound_approach_disabled'):
                self.distance_from_goal = self.get_distance(self.t_goal,self.t_base)
                if self.distance_from_goal < self.sovereignty_dist + 0.1:
                    self.sovereign = True
                else:
                    self.sovereign = False
                    self.time = rospy.get_time()
            elif self.distance_from_goal < self.sovereignty_dist:
                if not self.sovereign and rospy.get_time() - self.last_sov_time > sovtimelimit:
                    self.sovereign = True
                    self.last_sov_time = rospy.get_time()
            else:
                if self.sovereign and rospy.get_time() - self.last_sov_time > sovtimelimit:
                    self.sovereign = False
                    self.last_sov_time = rospy.get_time()
        else:
            self.sovereign = False

    def set_control(self,val):
        if val and not self.control:
            self.time_control = rospy.get_time()
            self.nav_vel_at_change = self.nav_vel
            self.control = True
        elif val:
            self.control = True
        elif not val:
            self.control = False
        else:
            self.control = False

    def get_status(self,t_base,phi_base,t_goal,phi_goal):
        self.distance_from_goal = self.get_distance(t_goal,t_base)

        self.set_sovereignty()
        self.set_stalled()
        
        vel_msg = Twist()
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        fb = GoLocationActionFeedback()
        # vel_msg is only used if control is set to True
        if self.sovereign and self.stalled:
            self.set_control(True)
            fb.feedback = GoLocationFeedback("sovereign and stalled (going to " + self.locid + ")")
            self.p_feedback.publish(fb)
            return vel_msg

        elif self.sovereign and not self.stalled:
            self.set_control(True)
            v,w = self.get_approach_path(t_base,phi_base,t_goal,phi_goal)
            vel_msg.linear.x = v
            vel_msg.angular.z = w
            fb.feedback = GoLocationFeedback("sovereign and not stalled (going to " + self.locid + ")")
            self.p_feedback.publish(fb)
            return vel_msg

        elif not self.sovereign and self.stalled:
            self.set_control(True)
            v,w = self.get_recovery_vel()
            vel_msg.linear.x = v
            vel_msg.angular.z = w
            fb.feedback = GoLocationFeedback("not sovereign and stalled: recovery (going to " + self.locid + ")")
            self.p_feedback.publish(fb)
            return vel_msg

        else:
            self.set_control(False)
            fb.feedback = GoLocationFeedback("move base in control (going to " + self.locid + ")")
            self.p_feedback.publish(fb)
            return None

    def get_recovery_vel(self):
        curtime = rospy.get_time()
        w = 0
        adiff = 0 
        if not self.path_point is None:
            t_, r_ = self.tf_listener.lookupTransform(self.base_link_frame,self.map_frame,rospy.Time(0))
            self.t_tobase =  np.array(t_)
            self.r_tobase = Rotation.from_quat(np.array(r_))
            pp = np.array([self.path_point.position.x,
                            self.path_point.position.y,
                            self.path_point.position.z])
            v_ =  self.r_tobase.apply(pp) + self.t_tobase
            adiff = np.arctan2(v_[1],v_[0])
        if not self.path_point is None: #and np.abs(adiff) > self.recovery_cone:
            # this tries to follow navigation
            # w = np.sign(adiff)*self.recovery_w
            w = np.sign(self.nav_w)*self.recovery_w
        else:
            # this tries to get away from the object
            left = False
            center = False
            right = False
            for i,d in enumerate(self.ultrasound_distances):
                if d < self.safety_distance:
                    if i < 2:
                        left = True
                    elif i > 2:
                        right = True
                    elif i == 2:
                        center = True
                    else:
                        print "error in recovery"
            if left and not right:
                w = -self.recovery_w
            elif not left and right:
                w = self.recovery_w
            elif center:
                w = (1 if np.random.uniform < 0.5 else -1)*self.recovery_w
        # if self.time_control - curtime < 0.1:
        #     if np.sign(self.time_control) != np.sign(w):
        #         w = -w
        self.last_recovery_w = w
        return 0,w

    def get_approach_path(self,t_base,phi_base,t_goal,phi_goal):
        maxv = 0.5
        maxphi = 0.5
        v0 = 0.05

        # This is to make a smooth transition between the two phases
        dt_control = rospy.get_time() - self.time_control 
        if dt_control < self.control_tau:
            v = (v0 - self.nav_vel_at_change)/self.control_tau*dt_control + self.nav_vel_at_change
        else:
            v = v0
        dp_ = phi_goal - phi_base
        if np.abs(dp_) > np.pi:
            # if we are not facing the good way we should leave 
            # move_base to do the magic 
            # TODO: our own algo could be possible better
            self.set_control(False)
            return None, None
            
        sign_ = np.sign(dp_)
        m1 = np.tan(phi_base)
        m2 = np.tan(phi_goal)
        x1 = t_base[0]
        y1 = t_base[1]
        x2 = t_goal[0]
        y2 = t_goal[1]

        if m2 == m1:
            w_ = 0
        else:
            x_ = (m1*m2*(y1 - y2) + m2*x1-m1*x2)/(m2-m1)
            y_ = 1/m2*(x2-x_) + y2
            r = np.sqrt((1+1/(m2*m2))*(x2 - x_)*(x2 - x_))
            w_ = v/r

        if np.isnan(v):
            v = 0
        if v > 0.25:
            v = 0.25

        if np.isnan(w_):
            w_ = 0
        if w_ < -0.25:
            w_  = -0.25
        elif w_ > 0.25:
            w_ = 0.25
        return v, sign_*w_


    

    def approach_table(self):
        
        goal = rospy.get_param('/biscee_location/' + self.locid)
        try:
            loctype = goal["loctype"]
        except:
            loctype = "freestanding"
        if loctype == "freestanding":
            self.sovereignty_allowed = False
        elif loctype == "closerange":
            self.sovereignty_allowed = True
        else:
            self.sovereignty_allowed = False
        self.t_goal = goal["trans"]
        self.r_goal = goal["rot"]

        if rospy.get_param('/biscee_tables/is_ultrasound_approach_disabled'):
            self.t_base,self.r_base = self.tf_listener.lookupTransform(self.map_frame,self.base_link_frame,rospy.Time(0))
            self.set_sovereignty()
            dt_ = rospy.get_time() - self.time
            if  self.sovereign and dt_ > 60:
                # we are very likely stuck
                print "oscilation problem"
                return "success"

            return "running"

        try:
            self.t_base,self.r_base = self.tf_listener.lookupTransform(self.map_frame,self.base_link_frame,rospy.Time(0))
            self.phi_base = self.angle_from_quat(self.r_base)
            self.phi_goal = self.angle_from_quat([0,0] + self.r_goal)
            dt_ = rospy.get_time() - self.time
            if dt_ > 30:
                print "timeout aborting"
                return "aborted"
            # print "sov", self.sovereign, "control",self.control
            self.vel_msg = self.get_status(self.t_base,self.phi_base,self.t_goal,self.phi_goal)

            if self.sovereign and self.distance_from_goal < 0.05:
                # if we are close let us be there
                print "goal"
                return "success"
            if self.sovereign and dt_ > 10:
                # we are very likely stuck
                print "premature goal"
                return "success"

            if self.sovereign and dt_ > 0.5 and self.distance_from_goal < 0.5 and self.ultrasound_distances[2] < self.safety_distance:
                print "likely table"
                return "success"

            if self.control:
                self.cmd_vel_pub.publish(self.vel_msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        return "running"

    def execute(self,userdata):
        print "approach",userdata.locid
        self.time = rospy.get_time()
        self.stalled = False
        self.locid = userdata.locid
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return "preempted"
            val = self.approach_table()
            if val == "success":
                return "success"
            elif val == "aborted":
                return "aborted"
            rate.sleep()
        fb = GoLocationActionFeedback()
        fb.feedback = GoLocationFeedback("not sovereign and stalled: recovery (going to " + self.locid + ")")
        self.p_feedback.publish(fb)
        return "preempted"


def approach_child_term_cb(outcome_map):

    print "calling child term",outcome_map
    nav = rospy.get_param('/biscee_tables/navigation_server')
    client = actionlib.SimpleActionClient(nav,MoveBaseAction)
    client.wait_for_server()
    client.cancel_all_goals()

    # if outcome_map['GoLocation']:
    #     return True

    # if outcome_map['ApproachTable']:
    #     return True

    # anything terminates, we should just terminate

    return True

def create_approach_table():
    navigation_server = rospy.get_param("/biscee_tables/navigation_server")
    ApproachTableCC = smach.Concurrence(
            outcomes = ["golocationsuccess",
                        "recoveryloc",
                        "recoverytab",
                        "preempted",
                        "approachsuccess"],
            default_outcome = "recoveryloc",
            input_keys = ['target_pose','GoLocationActionFeedback','locid'],
            output_keys = ['locid'],
            outcome_map = {
                "golocationsuccess": {'GoLocation':'succeeded'},
                "approachsuccess": {'ApproachTable':'success'},
                "recoveryloc": {'GoLocation':'aborted'},
                "recoverytab": {'ApproachTable':'aborted'},
                "preempted": {'ApproachTable':'preempted','GoLocation':'preempted'},
                },
            child_termination_cb = approach_child_term_cb
            )

    with ApproachTableCC:
        smach.Concurrence.add('ApproachTable',ApproachTable())
        smach.Concurrence.add('GoLocation',
                    SimpleActionState(navigation_server,
                                        MoveBaseAction,
                                        goal_slots = ["target_pose"],
                                        output_keys = ['locid'],
                                        input_keys = ['locid'],
                        )
                )
    return ApproachTableCC
