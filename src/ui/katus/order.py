import rclpy
from rclpy.node import Node
from serving_bot_interfaces.srv import PlaceOrder  # GUI.py에서 사용
from serving_bot_interfaces.msg import Order
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy


class MiddleNode(Node):
    def __init__(self):
        super().__init__('middle_node')
        
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 데이터 손실 없이 안정적으로 전송
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, # 구독자가 서버와 연결된 후 그 동안 수집된 데이터를 받을 수 있음
            history=QoSHistoryPolicy.KEEP_LAST, # 최근 메시지만 유지
            depth=10  # 최근 10개의 메시지를 유지
        )

        # GUI.py와 통신할 서비스
        self.order_client = self.create_client(PlaceOrder, '/order', qos_profile=self.qos_profile)

    def send_order_to_tabla_order(self, table_id, orders):
        """tabla order.py로 주문 데이터를 전달."""
        request = PlaceOrder.Request()

        request.table_id = table_id
        
        for k, v in orders.items():
            order = Order()
            order.menu_id = k
            order.quantity = v
            request.orders.append(order)
        
        print("order_client call async")
        future = self.order_client.call_async(request)

        print("spin start")
        rclpy.spin_until_future_complete(self, future)

        print("spin end")

        if future.result() is not None:
            response = future.result()
            if response.success:
                return True
            else:
                return False
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
