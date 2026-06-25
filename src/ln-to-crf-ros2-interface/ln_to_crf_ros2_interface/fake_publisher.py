import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OneShotPublisher(Node):

    def __init__(self):
        super().__init__('one_shot_publisher')

        self.publisher = self.create_publisher(String, '/planner_result', 10)

        self.timer = self.create_timer(0.5, self.publish_once)
        self.published = False

    def publish_once(self):
        if self.published:
            return

        plan = """
        move a
        move b
        move a d
        """

        cleaned_plan_lines = [
            line.strip() for line in plan.splitlines()
            if line.strip() and not line.strip().startswith(';')
        ]

        formatted_plan = '\n'.join(cleaned_plan_lines)

        msg = String()
        msg.data = formatted_plan

        self.publisher.publish(msg)
        self.get_logger().info("Published once!")

        self.published = True

        self.destroy_timer(self.timer)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = OneShotPublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()