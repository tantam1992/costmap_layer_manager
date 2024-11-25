#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CostmapLayerManager:
    def __init__(self):
        rospy.init_node('costmap_layer_manager', anonymous=True)

        # Subscribe to the state topic
        rospy.Subscriber('/fold_state', String, self.state_callback)

        # Publisher to stop the robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        try:
            # Initialize dynamic reconfigure clients for global and local costmaps
            self.global_unloaded_client = dynamic_reconfigure.client.Client('move_base/global_costmap/obstacle_layer_unloaded', timeout=30)
            self.global_loaded_client = dynamic_reconfigure.client.Client('move_base/global_costmap/obstacle_layer_loaded', timeout=30)

            self.local_unloaded_client = dynamic_reconfigure.client.Client('move_base/local_costmap/obstacle_layer_unloaded', timeout=30)
            self.local_loaded_client = dynamic_reconfigure.client.Client('move_base/local_costmap/obstacle_layer_loaded', timeout=30)

            rospy.loginfo("Costmap Layer Manager Node Initialized Successfully")
        except Exception as e:
            rospy.logerr(f"Failed to initialize dynamic reconfigure clients: {e}")
            raise

    def state_callback(self, msg):
        """Callback for state updates."""
        current_state = msg.data
        rospy.loginfo(f"Current fold state: {current_state}")

        # Check for specific states and enable layers accordingly
        if current_state == "OPERATIONAL/LOADED":
            rospy.loginfo("Enabling loaded layers and disabling unloaded layers")
            self.reconfigure_layers(loaded_enabled=True)
        else:
            rospy.loginfo("Enabling unloaded layers and disabling loaded layers")
            self.reconfigure_layers(loaded_enabled=False)

    def reconfigure_layers(self, loaded_enabled):
        """Reconfigure the obstacle layers based on the given state."""
        try:
            # Stop the robot before reconfiguring
            # self.stop_robot()

            # Disable layers first
            rospy.loginfo("Disabling current layers")
            self.global_unloaded_client.update_configuration({'enabled': not loaded_enabled})
            self.local_unloaded_client.update_configuration({'enabled': not loaded_enabled})

            # rospy.sleep(1.0)  # Increased delay

            # Enable the new layers
            rospy.loginfo("Enabling new layers")
            self.global_loaded_client.update_configuration({'enabled': loaded_enabled})
            self.local_loaded_client.update_configuration({'enabled': loaded_enabled})

            rospy.loginfo(f"Reconfiguration complete: loaded Enabled = {loaded_enabled}")
        except Exception as e:
            rospy.logerr(f"Failed to reconfigure layers: {e}")

    def stop_robot(self):
        """Continuously send zero velocity commands to stop the robot."""
        rate = rospy.Rate(10)  # 10 Hz publishing rate
        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        manager = CostmapLayerManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Costmap Layer Manager Node Shutting Down")
