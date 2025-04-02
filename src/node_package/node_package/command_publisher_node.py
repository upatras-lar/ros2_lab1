import rclpy
import random
from rclpy.node import Node
from interfaces_package.msg import JerryCommand

class CommandPublisher(Node):
    # A ROS2 node that publishes a JerryCommand

    def __init__(self):
        super().__init__('command_publisher_node')

        self.jerry_command_publisher = self.create_publisher(
            msg_type=JerryCommand,
            topic='/jerry_command',
            qos_profile=1
        )

        timer_period: float = 2.0
        self.timer = self.create_timer(timer_period, self.publish_command)

        self.incremental_id: int = 0
    
    def publish_command(self):
        # Method that is periodically called by the timer

        jerry_command = JerryCommand()

        p = random.random()
        if (p < 0.25):
            jerry_command.cmd = "forward"
        elif (p < 0.5):
            jerry_command.cmd = "right"
        elif (p < 0.75):
            jerry_command.cmd = "left"
        else:
            jerry_command.cmd = "backward"
        
        jerry_command.steps = random.randint(1, 5)

        self.jerry_command_publisher.publish(jerry_command)

        self.get_logger().info(f"Your next command Jerry is : {jerry_command.cmd} for {jerry_command.steps} steps.")

        self.incremental_id += 1

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but instead by ROS2 to configure
    certain aspects of the Node.
    """

    try:
        rclpy.init(args=args)

        jerry_command_publisher_node = CommandPublisher()

        rclpy.spin(jerry_command_publisher_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()