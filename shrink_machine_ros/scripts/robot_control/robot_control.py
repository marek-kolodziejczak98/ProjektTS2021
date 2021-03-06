import roslib
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from transforms3d import quaternions
import tf
import tf.transformations as tr
import numpy as np


class RobotControl:

    state_to_gripper_pose = {"Robot_in_base_position": [[0.25, 0.25, 0.4], [0.707, 0.707, 0.0, 0.0]],
                             "Robot_ready": [[0.35, 0.35, 0.15], [0.707, 0.707, 0.0, 0.0]],
                             "Robot_holding_beer": [[0.35, 0.35, 0.15], [0.707, 0.707, 0.0, 0.0]],
                             "Robot_in_final_position": [[0.1, 0.4, 0.3], [0.707, 0.707, 0.0, 0.0]],
                             "Return_to_shrink_process": [[0.25, 0.25, 0.4], [0.707, 0.707, 0.0, 0.0]],
                             "Station_waiting": [[0.25, 0.25, 0.4], [0.707, 0.707, 0.0, 0.0]]}

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ik_moveit")

        self.pub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal,
                              queue_size=1)

        self.compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        rospy.wait_for_service("/compute_ik")

        self.arm_move_group = moveit_commander.MoveGroupCommander("manipulator")
        self.robot_commander = moveit_commander.RobotCommander()

    def move_robot(self, current_state):

        if current_state in self.state_to_gripper_pose:
            print(f"Moving robot to the pose p={self.state_to_gripper_pose[current_state][0]},"
                  f" q={self.state_to_gripper_pose[current_state][1]}")

            # set reference position of the gripper ( link)
            pose_ik = PoseStamped()
            pose_ik.header.frame_id = "base_link"
            pose_ik.header.stamp = rospy.Time.now()
            pose_ik.pose = self.pq_to_pose(*self.state_to_gripper_pose[current_state])

            req = GetPositionIKRequest()
            req.ik_request.group_name = "manipulator"
            req.ik_request.robot_state = self.robot_commander.get_current_state()
            req.ik_request.avoid_collisions = True
            req.ik_request.ik_link_name = self.arm_move_group.get_end_effector_link()
            req.ik_request.pose_stamped = pose_ik
            ik_response = self.compute_ik_srv(req)

            if ik_response.error_code.val == 1:
                goal_pose = FollowJointTrajectoryActionGoal()
                # print (arm_move_group.get_joints())
                goal_pose.goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                                         'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
                goal_pose.goal.trajectory.points.append(JointTrajectoryPoint())
                goal_pose.goal.trajectory.points[0].positions = ik_response.solution.joint_state.position
                goal_pose.goal.trajectory.points[0].velocities = [0, 0, 0, 0, 0, 0]
                goal_pose.goal.trajectory.points[0].time_from_start = rospy.Duration.from_sec(1.0)
                self.pub.publish(goal_pose)
            else:
                print('Could not find solution to inverse kinematic')

    @staticmethod
    def pose_to_matrix(pose):
        """geometry_msgs.msg.Pose to 4x4 matrix"""
        transformer = tf.TransformerROS(True, rospy.Duration(1.0))
        p = [pose.position.x, pose.position.y, pose.position.z]
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        norm = np.linalg.norm(q)
        if np.abs(norm - 1.0) > 1e-3:
            raise ValueError(
                "Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(
                    str(q), np.linalg.norm(q)))
        elif np.abs(norm - 1.0) > 1e-6:
            q = q / norm
        g = transformer.fromTranslationRotation(p, q)
        return g

    @staticmethod
    def pq_to_pose(p, q):
        pose = Pose()
        pose.position.x = p[0]
        pose.position.y = p[1]
        pose.position.z = p[2]
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    def matrix_to_pose(self, mat):
        pose = Pose()
        quat = quaternions.mat2quat(mat[0:3, 0:3])
        quat_prim = [quat[1], quat[2], quat[3], quat[0]]
        pose = self.pq_to_pose([mat[0, 3], mat[1, 3], mat[2, 3]], quat_prim)
        return pose

