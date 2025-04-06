import Sofa.Core

from sofasurgsim.interfaces.ros_interface import ROSClient
from config.base_config import config as cfg

class RobotManager(Sofa.Core.Controller):
    def __init__(self, *args, ros_client: ROSClient, robot_node, **kwargs):
        super().__init__(*args, **kwargs)
        self.ros_client = ros_client
        self.robot_node = robot_node
        self.latest_joint_command = None  # Store incoming ROS commands

        # Subscribe to ROS joint targets
        self.ros_client.subscribe(
            cfg.ROBOT_JOINT_TOPIC,
            cfg.ROBOT_JOINT_TOPIC_TYPE,
            self._update_joint_command
        )

    def _update_joint_command(self, msg):
        """Callback for ROS joint commands"""
        self.latest_joint_command = msg

    def onAnimateBeginEvent(self, event):
        """Apply latest ROS command at start of simulation step"""
        if self.latest_joint_command:
            self._apply_joint_update(self.latest_joint_command)

    def _apply_joint_update(self, msg):
        """Update SOFA's joint positions to match ROS command"""
        joint_dofs = self.robot_node.getObject('arm_joints_dofs')
        for name, position in zip(msg.name, msg.position):
            idx = self._get_joint_index(name)  # Map joint name to DOF index
            joint_dofs.position[idx] = position