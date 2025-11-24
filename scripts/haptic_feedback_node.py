#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class HapticGuidanceNode:
    def __init__(self):
        rospy.init_node('haptic_feedback_node')



        self.target_j1 = 0.0  # Target for j1 in radians
        self.target_j2 = 0.0 # Target for j2 in radians

        # This gain controls how *strongly* the joystick guides you.
        # START WITH A SMALL VALUE!
        self.K_guidance = 0.5 

        # We need to map j1/j2 to pitch/roll. This is a guess.
        # You may need to swap them.
        self.j1_maps_to = 'roll' # 'roll' or 'pitch'

        # --- Internal Variables ---
        self.current_j1_pos = 0.5
        self.current_j2_pos = 1.0
        self.joint_state_received = False

        # --- ROS Publishers ---
        # Publishers for the gimbal controllers
        self.pitch_pub = rospy.Publisher('/pitch_effort_controller/command', Float64, queue_size=10)
        self.roll_pub = rospy.Publisher('/roll_effort_controller/command', Float64, queue_size=10)

        # --- ROS Subscriber ---
        # Subscriber to the robot's joint states
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

        rospy.loginfo("Haptic Guidance Node started.")

    def joint_states_callback(self, msg):
        """This function is called every time a new /joint_states message is received."""
        try:
            # Find the index of the joints
            j1_index = msg.name.index('j1')
            j2_index = msg.name.index('j2')

            # Read the position
            self.current_j1_pos = msg.position[j1_index]
            self.current_j2_pos = msg.position[j2_index]

            self.joint_state_received = True
        except ValueError:
            # If joints aren't found in the message, do nothing
            pass

    def run_loop(self):
        """The main 100Hz loop for calculating and publishing."""
        rate = rospy.Rate(100) # 100 Hz

        # Create message objects to re-use
        pitch_setpoint_msg = Float64()
        roll_setpoint_msg = Float64()

        while not rospy.is_shutdown():
            if not self.joint_state_received:
                rospy.logwarn_throttle(1.0, "Waiting for /joint_states message...")
                rate.sleep()
                continue

            # --- This is the core guidance logic ---

            # 1. Calculate arm error
            error_j1 = self.target_j1 - self.current_j1_pos
            error_j2 = self.target_j2 - self.current_j2_pos

            # 2. Convert arm error to joystick setpoint
            # We also cap the setpoint to the joint limits (-0.6 to 0.6)
            guidance_j1 = self.K_guidance * error_j1
            guidance_j2 = self.K_guidance * error_j2

            setpoint_1 = max(-0.6, min(0.6, guidance_j1))
            setpoint_2 = max(-0.6, min(0.6, guidance_j2))

            # 3. Map j1/j2 guidance to pitch/roll publishers
            if self.j1_maps_to == 'roll':
                roll_setpoint_msg.data = setpoint_1
                pitch_setpoint_msg.data = setpoint_2
            else:
                roll_setpoint_msg.data = setpoint_2
                pitch_setpoint_msg.data = setpoint_1

            # 4. Publish the new setpoints
            # The PID controller in ros_control will do the rest
            self.pitch_pub.publish(pitch_setpoint_msg)
            self.roll_pub.publish(roll_setpoint_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        node = HapticGuidanceNode()
        node.run_loop()
    except rospy.ROSInterruptException:
        pass