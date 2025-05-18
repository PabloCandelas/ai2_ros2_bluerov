import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String  # Import the String message type
from std_msgs.msg import Float64MultiArray
from stable_baselines3 import PPO
import pickle
import os
import numpy as np

class PPOControlNode(Node):
    def __init__(self):
        super().__init__('ppo_control_node')

        # Load the PPO model and VecNormalize
        self.get_logger().info("Loading PPO model and VecNormalize...")
     
        print(f"NumPy version: {np.__version__}")
        print(os.getcwd())
        domain_id = os.getenv("ROS_DOMAIN_ID", "0")
        print(f"ROS 2 Domain ID: {domain_id}")
        RMW_IMPLEMENTATION = os.getenv("RMW_IMPLEMENTATION", "0")
        print(f"ROS 2 RMW_IMPLEMENTATION: {RMW_IMPLEMENTATION}")
        # Paths to your trained PPO model and normalization files
        ppo_model_path = 'src/autonomous_rov/autonomous_rov/policies_ppo/bluerov_ppo_scratch3'
        #vecnorm_path = '/home/pablo/ros2_ws/src/autonomous_rov/autonomous_rov/policies_ppo/bluerov_vec_normalize_clean.pkl'

        # Load PPO model
        self.model = PPO.load(ppo_model_path)
        self.get_logger().info("PPO model loaded successfully!")

        # Load VecNormalize
        #with open(vecnorm_path, 'rb') as f:
        #    self.vecnorm = pickle.load(f)
        #self.get_logger().info("VecNormalize loaded successfully!")

        # Initialize variables for sensor data
        self.sensor_data = None  # This will hold the latest sensor data

        # Set up ROS2 subscriber to receive sensor data
        # Create the subscription to listen to the mock observation data
        self.subscriber = self.create_subscription(
            String,  # Expecting a String message, which is typically how JSON data is sent
            'observations',  # The topic that publishes the mock sensor data
            self.sensor_data_callback,  # Callback function to handle incoming messages
            10  # Queue size
        )
        # Publisher to send actions to control the robot
        self.publisher = self.create_publisher(Float64MultiArray, 'robot_command', 10)

        # Timer to simulate action taking at regular intervals
        self.timer = self.create_timer(1.0, self.take_action)  # 1.0 second interval

    def sensor_data_callback(self, msg):
        print("I AM ALIVEEEEEE!!!!")
        try:
            # Parse JSON string into a Python dictionary
            observation_dict = json.loads(msg.data)

            # Convert each value in the dictionary to a NumPy array of shape (1,)
            formatted_observation = {
                k: np.array([v], dtype=np.float32) for k, v in observation_dict.items()
            }

            # Store as plain dictionary (DO NOT wrap in np.array!)
            self.sensor_data = formatted_observation

            # Log to verify
            self.get_logger().info(f"Received and formatted observation: {self.sensor_data}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode observation JSON: {e}")
            self.sensor_data = None


    def take_action(self):
        """
        Take an action using the PPO model and publish it to the robot.
        """
        print("publishing")
        msg = Float64MultiArray()
        msg.data = [1.0]  # or whatever array of floats you want to send
        self.publisher.publish(msg)
        if self.sensor_data is not None:
            # Normalize the sensor data using VecNormalize
            #normalized_obs = self.vecnorm.normalize_obs(self.sensor_data)
            normalized_obs = self.sensor_data
            # Get action from PPO model based on the normalized observation
            action, _states = self.model.predict(normalized_obs)

            # Publish action to control the robot
            msg = Float64MultiArray()
            msg.data = action.flatten().tolist()  # Convert action to a list
            self.publisher.publish(msg)

            # Log the action taken (for debugging)
            self.get_logger().info(f"Action taken: {msg.data}")
        else:
            self.get_logger().warn("No sensor data received yet. Waiting for data...")

def main(args=None):
    rclpy.init(args=args)
    node = PPOControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
