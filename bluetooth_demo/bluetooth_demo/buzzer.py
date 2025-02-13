from ros_robot_controller_msgs.msg import BuzzerState


buzzer_msg = BuzzerState()
buzzer_msg.freq = 3200
buzzer_msg.on_time = 0.2
buzzer_msg.off_time = 0.01
buzzer_msg.repeat = 1

self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)

self.buzzer_pub.publish(buzzer_msg)