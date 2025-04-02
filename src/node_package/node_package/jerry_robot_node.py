import math
import rclpy
from rclpy.node import Node
from interfaces_package.msg import JerryCommand
from interfaces_package.srv import DistanceFromObstacle

class JerryRobot(Node):
    # A ROS2 node that represents Jerry the robot

    def __init__(self):
        super().__init__('jerry_robot_node')

        # Initialize Jerry at (x, y) = (0, 0)
        self.x = 0.
        self.y = 0.

        self.jerry_subscriber = self.create_subscription(
            msg_type=JerryCommand,
            topic='/jerry_command',
            callback=self.execute_command,
            qos_profile=1
        )

        self.jerry_server = self.create_service(
            srv_type=DistanceFromObstacle,
            srv_name='/distance',
            callback=self.service_callback)

    def execute_command(self, msg: JerryCommand):
        # Method that is called when a new command is received by Jerry.
        self.get_logger().info(f"Aye aye captain! Moving {msg.cmd} for {msg.steps} steps.")

        # Update Jerry's position
        if (msg.cmd == "forward"):
            self.y += msg.steps
        elif (msg.cmd == "backward"):
            self.y -= msg.steps
        elif (msg.cmd == "right"):
            self.x += msg.steps
        else:
            self.x -= msg.steps


    def service_callback(self, request: DistanceFromObstacle.Request,
                          response: DistanceFromObstacle.Response) -> DistanceFromObstacle.Response :
        """
        This function is called to respond to a DistanceFromObstacle request. 
        It takes the x and y coordinates of the obstalce as an input and calculates
        the Euclidean distance from Jerry's current whereabouts.
        """
        response.dist = math.dist([request.x, request.y], [self.x, self.y])

        self.get_logger().info(f"Distance from the obstacle is: {response.dist}. Responding to the client...")

        return response

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but instead by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        jerry_robot_node = JerryRobot()

        rclpy.spin(jerry_robot_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ =='__main__':
    main()