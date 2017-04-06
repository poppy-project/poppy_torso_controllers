#!/usr/bin/env python
from os.path import join
from rospkg import RosPack
from threading import Thread, RLock
import json
import rospy

from poppy.creatures import PoppyTorso
from poppy_msgs.srv import ExecuteTrajectory, SetCompliant, ExecuteTrajectoryResponse, SetCompliantResponse, ReachTarget, ReachTargetResponse
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


class TorsoControllers(object):
    def __init__(self, robot_name):
        """
        :param robot_name: Robot name and ROS topics/services namespace
        """
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('poppy_torso_controllers'), 'config', 'torso.json')) as f:
            self.params = json.load(f)

        self.publish_rate = rospy.Rate(self.params['publish_rate'])
        self.robot_name = robot_name

        self.eef_pub_l = rospy.Publisher('left_arm/end_effector_pose', PoseStamped, queue_size=1)
        self.eef_pub_r = rospy.Publisher('right_arm/end_effector_pose', PoseStamped, queue_size=1)
        self.js_pub_l = rospy.Publisher('left_arm/joints', JointState, queue_size=1)
        self.js_pub_r = rospy.Publisher('right_arm/joints', JointState, queue_size=1)
        self.js_pub_full = rospy.Publisher('joint_state', JointState, queue_size=1)

        # Services
        self.srv_execute = None
        self.srv_reach = None
        self.srv_left_arm_set_compliant = None
        self.srv_right_arm_set_compliant = None
        self.srv_robot_set_compliant = None

        # Protected resources
        self.torso = None
        self.robot_lock = RLock()

    def run(self, simulator=None):
        rospy.loginfo("Controller is connecting to {}...".format(self.robot_name))
        port = rospy.get_param('vrep/port', 19997)
        try:
            self.torso = PoppyTorso(use_http=True, simulator=simulator, scene="keep-existing", port=port)
        except IOError as e:
            rospy.logerr("{} failed to init: {}".format(self.robot_name, e))
            return None
        else:
            self.torso.compliant = False

            ########################## Setting up services
            self.srv_execute = rospy.Service('execute', ExecuteTrajectory, self._cb_execute)
            self.srv_reach = rospy.Service('reach', ReachTarget, self._cb_reach)

            self.srv_left_arm_set_compliant = rospy.Service('left_arm/set_compliant',
                                                            SetCompliant,
                                                            lambda req: self._cb_set_compliant(req, self.torso.l_arm))

            self.srv_right_arm_set_compliant = rospy.Service('right_arm/set_compliant',
                                                             SetCompliant,
                                                             lambda req: self._cb_set_compliant(req, self.torso.r_arm))

            self.srv_robot_set_compliant = rospy.Service('full_robot/set_compliant',
                                                         SetCompliant,
                                                         lambda req: self._cb_set_compliant(req, self.torso.motors))

            rospy.loginfo("{} controllers are up!".format(self.robot_name))

            while not rospy.is_shutdown():
                self.publish_eef(self.torso.l_arm_chain.end_effector, self.eef_pub_l)
                self.publish_eef(self.torso.r_arm_chain.end_effector, self.eef_pub_r)
                self.publish_js(self.torso.l_arm, self.js_pub_l)
                self.publish_js(self.torso.r_arm, self.js_pub_r)
                self.publish_js(self.torso.motors, self.js_pub_full)
                self.publish_rate.sleep()
        finally:
            if self.torso is not None:
                self.torso.compliant = True
                self.torso.close()

    @staticmethod
    def publish_eef(eef_pose, publisher):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'torso_base'
        pose.pose.position.x = eef_pose[0]
        pose.pose.position.y = eef_pose[1]
        pose.pose.position.z = eef_pose[2]
        publisher.publish(pose)

    def publish_js(self, arm, publisher):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = [m.name for m in arm]
        js.position = [m.present_position for m in arm]
        js.velocity = [m.present_speed for m in arm]
        js.effort = [m.present_load for m in arm]
        publisher.publish(js)

    def _cb_execute(self, request):
        # TODO Action server
        thread = Thread(target=self.execute, args=[request.trajectory])
        thread.daemon = True
        thread.start()
        return ExecuteTrajectoryResponse()

    def execute(self, trajectory):
        with self.robot_lock:
            rospy.loginfo("Executing Torso trajectory with {} points...".format(len(trajectory.points)))
            time = 0.
            for point_id, point in enumerate(trajectory.points):
                if rospy.is_shutdown():
                    break

                time_from_start = point.time_from_start.to_sec()
                duration = time_from_start - time

                if duration < 0.:
                    rospy.logwarn("Skipping invalid point {}/{} with incoherent time_from_start", point_id + 1, len(trajectory.points))
                    continue

                self.torso.goto_position(dict(zip(trajectory.joint_names, point.positions)),
                                         self.params['time_margin'] + duration)  # Time margin trick to smooth trajectory
                sleep = max(0, duration - 0.00)
                rospy.sleep(sleep)
                time = time_from_start
            rospy.loginfo("Trajectory ended!")

    def _cb_set_compliant(self, request, arm):
        rospy.loginfo("{} now {}".format(self.robot_name, 'compliant' if request.compliant else 'rigid'))
        with self.robot_lock:
            for m in arm:
                m.compliant = request.compliant
        return SetCompliantResponse()

    def _cb_reach(self, request):
        target = dict(zip(request.target.name, request.target.position))
        with self.robot_lock:
            rospy.loginfo("Reaching non-blocking target...")
            self.torso.goto_position(target, request.duration.to_sec())
        return ReachTargetResponse()

if __name__ == '__main__':
    rospy.init_node("poppy_torso_controllers")
    simulator = rospy.get_param("simulator", None)
    if simulator == "none":
        simulator = None
    TorsoControllers(rospy.get_namespace().strip('/')).run(simulator=simulator)
