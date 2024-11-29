import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose

class tset(Node):
    def __init__(self):
        super().__init__('test')
        self.test_action = ActionServer(self, NavigateToPose, 'navigate_to_pose', self.execute_callback)
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal: moving to position {}'.format(goal_handle.request.pose))

        # 피드백을 준비합니다.
        result = NavigateToPose.Result()
        feedback_msg = NavigateToPose.Feedback()
        print('serve complete')

        # 결과 설정 (성공적으로 이동 완료)
        feedback_msg.success = True
        goal_handle.succeed()  # 작업 성공으로 마무리

        return result
    
def main(args=None):
    rclpy.init(args=args)
    action_server = tset()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()