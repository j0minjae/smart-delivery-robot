import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from serving_bot_interfaces.srv import PlaceOrder  # GUI.py에서 사용
from geometry_msgs.msg import PoseStamped


class MiddleNode(Node):
    def __init__(self):
        super().__init__('middle_node')
        # GUI.py와 통신할 서비스
        self.order_service = self.create_client(PlaceOrder, '/order', 10)

    def send_order_to_tabla_order(self, order_details):
        """tabla order.py로 주문 데이터를 전달."""
        request = Trigger.Request()
        request.message = "\n".join(order_details)

        future = self.order_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                return True, response.message
            else:
                return False, response.message
        else:
            return False, "No response from tabla order service."


def main():
    rclpy.init()

    middle_node = MiddleNode()

    try:
        rclpy.spin(middle_node)
    except KeyboardInterrupt:
        middle_node.get_logger().info('Shutting down middle node...')
    finally:
        middle_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
