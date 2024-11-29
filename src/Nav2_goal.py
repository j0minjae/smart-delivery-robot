import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time  # 대기 시간 추가를 위해 필요

        

# class GoalSender(Node):
#     def __init__(self):
class ControlManager:
    def __init__(self, gui_node:None):
        super().__init__()
        self.gui_node = gui_node
        self.gui_node._action_client = ActionClient(self.gui_node, NavigateToPose, 'navigate_to_pose')
        
        # 사전 정의된 목표 좌표
        self.set_nav2_pose = {
            '0': Point(x=0.0, y=0.0, z=0.0),  # 복귀 지점
            '1': Point(x=2.5, y=1.5, z=0.0),  # table_id = '1'
            '2': Point(x=2.538276433944702, y=0.48459047079086304, z=0.0),   # table_id = '2'
            '3': Point(x=2.532015562057495, y=-0.5978966951370239, z=0.0),   # table_id = '2'
            '4': Point(x=1.4370678663253784, y=1.59836745262146, z=0.0),   # table_id = '2'
            '5': Point(x=1.4370638132095337, y=.5033659338951111, z=0.0),   # table_id = '2'
            '6': Point(x=1.4327003955841064, y=-0.5833651423454285, z=0.0),   # table_id = '2'
            '7': Point(x=0.29234251379966736, y=1.5998566150665283, z=0.0),   # table_id = '2'
            '8': Point(x=0.2931201756000519, y=0.5048375725746155, z=0.0),   # table_id = '2'
            '9': Point(x=0.30012351274490356, y=0.002471923828125, z=0.0),   # table_id = '2'
        }
        self.current_goal = None  # 현재 목표 ID

    def send_goal(self, table_id):
        # table_id가 유효한지 확인
        if table_id not in self.set_nav2_pose:
            self.gui_node.get_logger().error(f"Invalid table_id: {table_id}")
            return
        
        # 목표 좌표 설정
        self.current_goal = table_id  # 현재 목표 설정
        target_pose = self.set_nav2_pose[table_id]
        
        # 목표 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.gui_node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_pose.x
        goal_msg.pose.pose.position.y = target_pose.y
        goal_msg.pose.pose.orientation.z = target_pose.z
        goal_msg.pose.pose.orientation.w = 1.0  # 방향 기본값

        # 서버 대기
        self.gui_node._action_client.wait_for_server()
        self.gui_node.get_logger().info(
            f"Sending goal to: x={target_pose.x}, y={target_pose.y}, theta={target_pose.z}"
        )
        
        # 목표 전송
        send_goal_future = self.gui_node._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.gui_node.get_logger().info(f"Current pose: {feedback.current_pose}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.gui_node.get_logger().error("Goal rejected :(")
            return
        
        self.gui_node.get_logger().info("Goal accepted :)")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result.status == 4:  # STATUS_SUCCEEDED
            self.gui_node.get_logger().info(f"Goal '{self.current_goal}' reached successfully!")
            
            # 목표 지점에서 7초 대기
            if self.current_goal != '0':  # 복귀 목표가 아니라면
                self.gui_node.get_logger().info("Waiting for 7 seconds before returning...")
                time.sleep(3)
                self.gui_node.get_logger().info("Switching to return goal (table_id='0').")
                self.send_goal('0')  # 복귀 지점으로 목표 전환
            else:
                self.gui_node.get_logger().info("Return goal reached. Shutting down.")
                # rclpy.shutdown()
        else:
            self.gui_node.get_logger().warn(f"Goal failed with status: {result.status}")
            # rclpy.shutdown()


# def main():
#     rclpy.init()
#     node = GoalSender()
#     try:
#         # node.send_goal('1')  # 첫 번째 목표 전송
#         # node.send_goal('2')
#         # node.send_goal('3')
#         # node.send_goal('4')
#         # node.send_goal('5')
#         # node.send_goal('6')
#         node.send_goal('7')
#         # node.send_goal('8')
#         # node.send_goal('9')

#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("KeyboardInterrupt, shutting down.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
