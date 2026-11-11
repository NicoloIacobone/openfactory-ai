"""ROS names used by openfactory_perception (docs/architecture.md)."""

WORLD_FRAME = 'world'
INPUT_BIN_FRAME = 'input_bin'
OPTICAL_FRAME = 'camera_optical_frame'

DEPTH_TOPIC = '/camera/depth/image_raw'
COLOR_TOPIC = '/camera/color/image_raw'
CAMERA_INFO_TOPIC = '/camera/camera_info'

OBJECTS_TOPIC = '/perception/objects'
DEBUG_POINTS_TOPIC = '/perception/debug/points'
DEBUG_IMAGE_TOPIC = '/perception/debug/image'

DESCRIPTION_PACKAGE = 'openfactory_description'
INPUT_BIN = 'input'                       # key of the bin in cell.yaml `bins`
UNKNOWN_CLASS = 'unknown'
