import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from serving_bot_interfaces.srv import PlaceOrder  # GUI.py에서 사용
from geometry_msgs.msg import PoseStamped


class MiddleNode(Node):
    def __init__(self):
        super().__init__('middle_node')

        # tabla order.py와 통신할 클라이언트
        self.order_client = self.create_client(Trigger, 'process_order')
        while not self.order_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('process_order 서비스 대기 중...')

        # GUI.py와 통신할 서비스
        self.order_service = self.create_service(PlaceOrder, '/order', self.handle_order_request)

    def handle_order_request(self, request, response):
        """GUI.py로부터 주문 데이터를 받고 tabla order.py로 전달."""
        table_id = request.table_id
        orders = request.orders

        self.get_logger().info(f"Received order from GUI: Table {table_id}, Orders: {orders}")

        # 주문 데이터를 Trigger 서비스로 전달
        order_details = [f"Table {table_id}: {item}" for item in orders]
        success, message = self.send_order_to_tabla_order(order_details)

        if success:
            self.get_logger().info(f"Order successfully sent to tabla order: {message}")
            response.success = True
            response.message = "Order processed successfully"
        else:
            self.get_logger().error(f"Failed to send order: {message}")
            response.success = False
            response.message = "Failed to process order"

        return response

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
