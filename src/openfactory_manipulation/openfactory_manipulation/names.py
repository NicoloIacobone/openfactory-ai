"""ROS names of one arm: its planning group, joints, links and gripper action.

The single-arm cell has one set (`CELL`, the names of `constants`); in the ring of N arms
(phase 6, D-020) every arm has its own, from openfactory_description.ring.
"""

from dataclasses import dataclass

from . import constants as c


@dataclass(frozen=True)
class ArmNames:
    group: str                    # MoveIt planning group of the arm
    joints: tuple                 # arm joints, base to wrist
    tool_link: str                # tip of the group, the frame of motion goals
    tcp_link: str                 # grasp point between the pads: held objects attach here
    touch_links: tuple            # gripper links allowed to touch a held object
    finger_joint: str             # commanded finger; the other one mimics it
    finger_joints: tuple
    gripper_action: str


CELL = ArmNames(
    group=c.ARM_GROUP, joints=c.ARM_JOINTS, tool_link=c.TOOL_LINK, tcp_link=c.TCP_LINK,
    touch_links=c.GRIPPER_TOUCH_LINKS, finger_joint=c.FINGER_JOINT,
    finger_joints=c.FINGER_JOINTS, gripper_action=c.GRIPPER_ACTION)


def ring_arm(index):
    """Names of arm `index` of the ring."""
    from openfactory_description import ring

    prefix = ring.arm_prefix(index)
    return ArmNames(
        group=ring.planning_group(index),
        joints=tuple(ring.arm_joints(index)),
        tool_link=prefix + c.TOOL_LINK,
        tcp_link=prefix + c.TCP_LINK,
        touch_links=tuple(prefix + link for link in c.GRIPPER_TOUCH_LINKS),
        finger_joint=prefix + c.FINGER_JOINT,
        finger_joints=tuple(ring.finger_joints(index)),
        gripper_action=f'/{ring.gripper_controller(index)}/gripper_cmd')
