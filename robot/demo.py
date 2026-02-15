import robot.abb_irc5 as irc5
import robot.sensors as sensors

if __name__ == "__main__":
    sensor_client = sensors.connect_sensors()
    sensors.start_sensors()
    robot = irc5.connect_robot()
    
    try:
        irc5.read_robot_state()
        print(f"Current Position: {irc5.robot_state.pos}, Orientation: {irc5.robot_state.orientation}")

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        print("Disconnecting from robot...")
        irc5.stop_robot()
        irc5.disconnect_robot()
        sensors.stop_sensors()