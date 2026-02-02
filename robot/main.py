import robot.abb_irc5 as irc5
import robot.sensors as sensors

if __name__ == "__main__":
    sensor_client = sensors.connect_sensors()
    sensors.start_sensors()
    robot = irc5.connect_robot()
    
    try:
        while True:
            irc5.receive_data()
            print(f"Robot Position: {irc5.robot_state.pos}")
            irc5.send_cartesian_command(0.0, 0.0, 1, 1.0, 0.0, 0.0, 0.0)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        sensors.stop_sensors()
        irc5.disconnect_robot()