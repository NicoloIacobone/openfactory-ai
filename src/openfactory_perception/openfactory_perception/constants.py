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

# every topic of scene_perception: a launch moves them into the namespace of a bin when the
# cell has one camera per bin (ring of N arms)
TOPICS = (DEPTH_TOPIC, COLOR_TOPIC, CAMERA_INFO_TOPIC, OBJECTS_TOPIC, DEBUG_POINTS_TOPIC,
          DEBUG_IMAGE_TOPIC)

# cube_perception (Rubik's cube cell, phase 7)
NEST_FRAME = 'nest'
CUBE_DETECTION_TOPIC = '/perception/cube/detection'
CUBE_FACE_TOPIC = '/perception/cube/top_face'
CUBE_DEBUG_TOPIC = '/perception/cube/debug/image'
CUBE_CLASS = 'rubik_cube'

DESCRIPTION_PACKAGE = 'openfactory_description'
INPUT_BIN = 'input'                       # key of the bin in cell.yaml `bins`
UNKNOWN_CLASS = 'unknown'
