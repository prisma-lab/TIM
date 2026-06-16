import rclpy

from ln_to_crf_ros2_interface.topic_manager import TopicManager

def main(args=None):
    rclpy.init(args=args)
    topic_mamanger = TopicManager()
    print("Topic manager ros node initialized!")
    rclpy.spin(topic_mamanger)
    topic_mamanger.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
