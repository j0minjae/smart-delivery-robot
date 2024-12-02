import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rcl_interfaces.msg import Log
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Pose
from serving_bot_interfaces.srv import PlaceOrder
from ui import DataManager
from DTO import OrderTicket, MenuItem, TableInfo
import math
from db_management import OrderManager
# from nav2_goal import Nav2Goal
import time 


class gui(Node):
    def __init__(self, db_instance, data_manager:DataManager=None):
        super().__init__('gui')

        self.get_logger().info("gui node init")

        self.db = db_instance
        self.data_manager = data_manager
        # self.nav2_goal = nav2_goal_node

        self.data_manager.robot_serve_update.connect(self.move_to_table)

        self.order_service = self.create_service(PlaceOrder,'/order', self.order_callback)
        self.log_sub = self.create_subscription(Log, '/rosout', self.log_callback, 10)
        # self.send_menu = ActionClient(self,NavigateToPose, 'navigate_to_pose')
        self.Nav2_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 사전 정의된 목표 좌표
        self.set_nav2_pose = {
            '0': Point(x=0.0, y=0.0, z=0.0),  # 복귀 지점
            '1': Point(x=2.5, y=1.5, z=0.0),  # table_id = '1'
            '2': Point(x=2.5, y=0.5, z=0.0),   # table_id = '2'
            '3': Point(x=2.5, y=-0.6, z=0.0),   # table_id = '3'
            '4': Point(x=1.45, y=1.5, z=0.0),   # table_id = '4'
            '5': Point(x=1.45, y=0.5, z=0.0),   # table_id = '5'
            '6': Point(x=1.45, y=-0.5, z=0.0),   # table_id = '6'
            '7': Point(x=0.3, y=1.5, z=0.0),   # table_id = '7'
            '8': Point(x=0.3, y=0.5, z=0.0),   # table_id = '8'
            '9': Point(x=0.3, y=-0.5, z=0.0),   # table_id = '9'
        }
        self.current_goal = None  # 현재 목표 ID

    # def euler_to_quaternion(self, roll, pitch, yaw):
    #     # Convert Euler angles to a quaternion
    #     qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    #     qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    #     qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    #     qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    #     return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def log_callback(self, msg):
        self.db.update_log(msg)
        
    def order_callback(self, request, response):
        self.get_logger().info(f'check')
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

    def move_to_table(self, table_id):
        self.get_logger().info(f"{table_id}로 이동")
        # self.contorl_manager.send_goal(str(table_id))
        self.send_goal(table_id)
        # self.nav2_goal.send_goal(table_id)

        # if table_id in self.locate_tables:
        #     target_pose = self.locate_tables[table_id]
        #     self.movement(target_pose)
    def send_goal(self, table_id):
        # table_id가 유효한지 확인
        if str(table_id) not in self.set_nav2_pose:
            self.get_logger().error(f"Invalid table_id: {table_id}")
            return
        
        # 목표 좌표 설정
        self.current_goal = table_id  # 현재 목표 설정
        target_pose = self.set_nav2_pose[str(table_id)]
        
        # 목표 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_pose.x
        goal_msg.pose.pose.position.y = target_pose.y
        goal_msg.pose.pose.orientation.z = target_pose.z
        goal_msg.pose.pose.orientation.w = 1.0  # 방향 기본값

        # 서버 대기
        self.Nav2_action_client.wait_for_server()
        self.get_logger().info(
            f"Sending goal to: x={target_pose.x}, y={target_pose.y}, theta={target_pose.z}"
        )
        
        # 목표 전송
        send_goal_future = self.Nav2_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f"Current pose: {feedback}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            return
        
        self.get_logger().info("Goal accepted :)")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info(f"Goal '{self.current_goal}' reached successfully!")
            
            # 목표 지점에서 5초 대기
            if self.current_goal != 0:  # 복귀 목표가 아니라면
                self.get_logger().info("Waiting for 7 seconds before returning...")
                time.sleep(5)
                self.get_logger().info("Switching to return goal (table_id='0').")
                self.send_goal(0)  # 복귀 지점으로 목표 전환 변경
            else:
                self.get_logger().info("Return goal reached. Shutting down.")
        else:
            self.get_logger().warn(f"Goal failed with status: {result.status}")
    
    # def movement(self, target_pose):
    #     movement = PoseStamped()
    #     movement.header.frame_id = 'map'  # SLAM에서 사용되는 좌표계 (보통 'map' 프레임)
    #     movement.header.stamp = self.get_clock().now().to_msg()
    #     movement.pose.position.x = target_pose.position.x
    #     movement.pose.position.y = target_pose.position.y
    #     movement.pose.orientation = target_pose.orientation
    #     self.send_menu.wait_for_server()
    #     self.get_logger().info(f'complete to serve')
    #     goal = NavigateToPose.Goal()
    #     goal.pose = movement
    #     self.send_menu.send_goal_async(
    #         goal,
    #         feedback_callback=self.feedback_callback
    #     ).add_done_callback(self.goal_response_callback)

    # def feedback_callback(self, feedback_msg):
    #     feedback = feedback_msg.feedback
    #     self.get_logger().info(f"Current progress: {feedback.current_pose}")

    # def goal_response_callback(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().info('Goal rejected :(')
    #         return
    #     self.get_logger().info('Goal accepted :)')
    #     goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    # def get_result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info(f'Result: {result}')

    def call_log(self):
        return self.db.call_log()
    
    def serve_button_callback(self):
        self.get_logger().info("serve button clicked in gui node")



# def run_ros(node):
#     rclpy.spin(node)

# def run_ui(ui_instance, app):
#     ui_instance.show()
#     app.exec_()
    
# def main(args=None):
#     rclpy.init(args=args)
#     # app = QApplication([])
#     # ui_instance = OrderUI()
#     db_instance = OrderManager()
#     node = gui(db_instance)
#     rclpy.spin(node)

#     # threading = Thread(target=run_ros, args=[node,], daemon=True)
#     # threading.start()
#     # run_ui(ui_instance, app)
    
#     # database 닫는거 db_instance.claise_database()

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()