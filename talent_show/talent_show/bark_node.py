import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BuzzerState
from std_srvs.srv import Trigger
import time

class BarkNode(Node):
    def __init__(self):
        super().__init__('bark_node')
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 10)
        self.create_service(Trigger, 'trigger_bark', self.bark_callback)
        self.get_logger().info('Bark node has been started')

        # Test Code to set up a client to self-trigger bark.
        self.client = self.create_client(Trigger, 'trigger_bark')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.request = Trigger.Request()

    # Test trigger bark function.
    def send_request(self):
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    def bark_callback(self, request, response):
        # Create a bark pattern (two-tone bark)
        self.bark_pattern()
        response.success = True
        return response

    def bark_pattern(self):
        buzzer_msg = BuzzerState()
        
        # First part of bark (higher pitch)
        buzzer_msg.freq = 650  # Higher frequency
        buzzer_msg.on_time = 0.15
        buzzer_msg.off_time = 0.05
        buzzer_msg.repeat = 1
        self.buzzer_pub.publish(buzzer_msg)
        time.sleep(1)

        # Second part of bark (lower pitch)
        buzzer_msg.freq = 600  # Lower frequency
        buzzer_msg.on_time = 0.2
        buzzer_msg.off_time = 0.05
        buzzer_msg.repeat = 1
        self.buzzer_pub.publish(buzzer_msg)
        time.sleep(0.8)

        # Second part of bark (lower pitch)
        buzzer_msg.freq = 630  # Lower frequency
        buzzer_msg.on_time = 0.2
        buzzer_msg.off_time = 0.05
        buzzer_msg.repeat = 1
        self.buzzer_pub.publish(buzzer_msg)

        time.sleep(3)

        self.play_rickroll()

    def play_rickroll(self):
        # Each note lasts 0.5 seconds so 30 notes ≈ 15 seconds total.
        note_duration = 0.5  # seconds per note
        
        # Define a simple melody using note names.
        # This sequence is arranged to roughly mimic the chorus:
        # "Never gonna give you up, never gonna let you down,
        #  never gonna run around and desert you, never gonna..."
        note_freq = {
            "A": 440,
            "B": 493,
            "C#": 554,
            "D": 587,
            "E": 659,
            "F#": 740,
            "^A": 880,
            "^B": 987,
            "^C#": 1108,
            "^D": 1174,
            "^E": 1318,
            "^F#": 1479
        }
        
        # Encode the transcription as a sequence of tokens.
        # Notes are given as in your transcription.
        # The "~" token represents a rest.
        melody = [
            # Phrase 1: "A-B      ^D-B    ^F# ^F# ^E"
            "A", "B", "^D", "B", "^F#", "^F#", "^E",
            # Phrase 2: "A-B      ^D-B  ^E  ^E  ^D-^C#-B" with a rest at the end ( ~ )
            "A", "B", "^D", "B", "^E", "^E", "^D", "^C#", "B", "~",
            # Phrase 3: "A-B      ^D-B   ^D   ^E-^C#    A     A-^E   ^D"
            "A", "B", "^D", "B", "^D", "^E", "^C#", "A", "A", "^E", "^D",
            # Phrase 4: "A-B       ^D-B    ^F#   ^F# ^E"
            "A", "B", "^D", "B", "^F#", "^F#", "^E",
            # Phrase 5: "A-B       ^D-B   ^A   ^C#-^D-^C#-B" with two rests after
            "A", "B", "^D", "B", "^A", "^C#", "^D", "^C#", "B", "~", "~",
            # Phrase 6: "A-B      ^D-B   ^D ^E ^C#-A   A   ^E   ^D" with a rest at the end
            "A", "B", "^D", "B", "^D", "^E", "^C#", "A", "A", "^E", "^D", "~"
        ]
        
        # Iterate over the melody tokens.
        for token in melody:
            if token == "~":
                # This is a rest: wait for the duration without playing a note.
                time.sleep(note_duration)
            else:
                # Look up the frequency for the note.
                frequency = note_freq.get(token, 500)  # default to 500Hz if not found
                
                # Create and configure your buzzer message.
                # (Replace BuzzerMessage with your actual message type as needed.)
                buzzer_msg = BuzzerState()
                buzzer_msg.freq = frequency
                # Play the note for 80% of the token duration; leave 20% as a gap.
                buzzer_msg.on_time = note_duration * 0.4
                buzzer_msg.off_time = note_duration * 0.01
                buzzer_msg.repeat = 1
                
                # Publish the message to the buzzer.
                self.buzzer_pub.publish(buzzer_msg)
                
                # Wait the full token duration before the next token.
                time.sleep(note_duration)



def main(args=None):
    rclpy.init(args=args)
    bark_node = BarkNode()
    # Test Code to run bark callback.
    # bark_node.send_request()
    rclpy.spin(bark_node)
    bark_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
