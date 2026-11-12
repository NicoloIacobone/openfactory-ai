"""ROS names used by openfactory_manipulation (docs/architecture.md)."""

WORLD_FRAME = 'world'
INPUT_BIN_FRAME = 'input_bin'
INPUT_BIN = 'input'                                     # key of the bin in cell.yaml
OUTPUT_BIN_FRAME = 'output_bin'
TCP_LINK = 'gripper_tcp'
TOOL_LINK = 'tool0'
GRIPPER_TOUCH_LINKS = ('gripper_palm', 'left_finger', 'right_finger')

ARM_GROUP = 'ur_manipulator'
ARM_JOINTS = (
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
)
FINGER_JOINT = 'left_finger_joint'                      # commanded; the right one mimics it
FINGER_JOINTS = ('left_finger_joint', 'right_finger_joint')

JOINT_STATES_TOPIC = '/joint_states'
MOVE_ACTION = '/move_action'
EXECUTE_ACTION = '/execute_trajectory'
CARTESIAN_SERVICE = '/compute_cartesian_path'
PLANNING_SCENE_SERVICE = '/apply_planning_scene'
IK_SERVICE = '/compute_ik'
GRIPPER_ACTION = '/gripper_controller/gripper_cmd'
OBJECTS_TOPIC = '/perception/objects'

DESCRIPTION_PACKAGE = 'openfactory_description'
PACKAGE = 'openfactory_manipulation'
