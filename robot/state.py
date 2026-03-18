from robot.globals import *

# --- STATES ---
class RobotState:
    def __init__(self, mode):
        self.motion = MotionState.IDLE
        self.three_point = ThreePointState.IDLE
        self.calibration = mode
        self.recorded_points = []

        if self.calibration == CalibrationMode.FOUR_POINT:
            camera_topic = ThreePointState.FIND_CENTER.value
        else:
            camera_topic = self.three_point.value

        self.subscriber = MQTTState(mqtt_broker=MQTT_BROKER, camera_topic=camera_topic)
        self.robot_config = RobotConfig(ip_address=ROBOT_REAL_IP)

    def set_target(self, target):
        self.three_point = target
        self.subscriber.camera_topic = self.three_point.value

# global_state.set_target(ThreePointState.FIND_CENTER)