import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rcl_interfaces.msg import Log
from rclpy.logging import LoggingSeverity
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Pose
from serving_bot_interfaces.srv import PlaceOrder
from ui import DataManager
from DTO import OrderTicket, MenuItem, TableInfo
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from db_management import OrderManager
from Nav2_goal import ControlManager


class gui(Node):
    def __init__(self, db_instance:OrderManager, data_manager:DataManager=None, enable_control_manager:bool=True):
        super().__init__('gui')

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 데이터 손실 없이 안정적으로 전송
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, # 구독자가 서버와 연결된 후 그 동안 수집된 데이터를 받을 수 있음
            history=QoSHistoryPolicy.KEEP_LAST, # 최근 메시지만 유지
            depth=10  # 최근 10개의 메시지를 유지
        )

        self.get_logger().info("gui node init")

        if enable_control_manager:
            self.control_manager = ControlManager(self)
        else:
            self.control_manager = None

        self.db = db_instance
        self.data_manager = data_manager

        self.data_manager.robot_serve_update.connect(self.move_to_table)
        self.data_manager.log_signal.connect(self.kitchen_display_emit_log)

        self.order_service = self.create_service(PlaceOrder,'/order', self.qos_profile, self.order_callback)
        self.log_sub = self.create_subscription(Log, '/rosout', self.log_callback, self.qos_profile)
        self.send_menu = ActionClient(self,NavigateToPose, 'navigate_to_pose')

        self.kitchen_display_logger = self.get_logger().get_child("kitchen_display")
        self.logger = self.get_logger()
    
    def kitchen_display_emit_log(self, message, level):
        if level == 10:
            # self.kitchen_display_logger.debug(message)
            self.get_logger().debug(message)
        elif level ==20:
            # self.kitchen_display_logger.info(message)
            self.get_logger().info(message)
        elif level ==30:
            # self.kitchen_display_logger.warn(message)
            self.get_logger().warn(message)
        elif level ==40:
            # self.kitchen_display_logger.error(message)
            self.get_logger().error(message)
        elif level ==50:
            # self.kitchen_display_logger.fatal(message)
            self.get_logger().fatal(message)

    def log_callback(self, msg:Log):
        print("log test")
        self.db.update_log(msg)
        self.data_manager.create_log(msg)
        

    def order_callback(self, request, response):
        try:
            table_id = request.table_id
            orders = request.orders
            self.db.update_db(table_id, orders)

            # if table_id in self.locate_tables:
            #     target_pose = self.locate_tables[table_id]
            #     self.movement(target_pose)

            # Data Manager
            if self.data_manager:
                orders = [MenuItem(menu.menu_id, menu.quantity) for menu in orders]
                ticket = OrderTicket(table_id, order=orders)
                self.data_manager.create_order(ticket)

            response.success = True
            
            return response
        except:
            self.get_logger().info(f'주문 시스템에 문제가 발생했습니다')    #send ui

    def move_to_table(self, table_id):
        self.get_logger().info(f"{table_id}로 이동")
        
        if self.control_manager:
            self.control_manager.send_goal(str(table_id))
    
    def movement(self, target_pose):
        movement = PoseStamped()
        movement.header.frame_id = 'map'  # SLAM에서 사용되는 좌표계 (보통 'map' 프레임)
        movement.header.stamp = self.get_clock().now().to_msg()
        movement.pose.position.x = target_pose.position.x
        movement.pose.position.y = target_pose.position.y
        movement.pose.orientation = target_pose.orientation
        self.send_menu.wait_for_server()
        self.get_logger().info(f'complete to serve')
        goal = NavigateToPose.Goal()
        goal.pose = movement
        self.send_menu.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Current progress: {feedback.current_pose}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')

    def call_log(self):
        return self.db.call_log()
    
    def serve_button_callback(self):
        self.get_logger().info("serve button clicked in gui node")
    
def main(args=None):
    rclpy.init(args=args)
    db_instance = OrderManager()
    node = gui(db_instance)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()