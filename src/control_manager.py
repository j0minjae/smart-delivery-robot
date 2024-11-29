import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from nav2_msgs.srv import SetInitialPose
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
import time  # 대기 시간 추가를 위해 필요

class InitialPose(Node):
    def __init__(self):
        super().__init__('navigation_node')  # Node 초기화
        self.num = 3
        self.init_pose = [0.0, 0.0, 0.0, 1.0]
        self.goal_poses = [[0.0, 0.0] for _ in range(self.num)]
        self.setting_poses = [False for _ in range(self.num)]

        # SetInitialPose 서비스 클라이언트 생성
        self.set_initial_pose_service_client = self.create_client(
            SetInitialPose,
            '/set_initial_pose'
        )

        self.initial_status = False
        # 서비스 대기
        while not self.set_initial_pose_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /set_initial_pose not available, waiting again...')

        # 초기 포즈 설정
        self.set_initial_pose(*self.init_pose)

    def set_initial_pose(self, x, y, z, w):
        req = SetInitialPose.Request()
        req.pose.header.frame_id = 'map'
        req.pose.pose.pose.position = Point(x=x, y=y, z=0.0)
        req.pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)
        req.pose.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.1,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]

        future = self.set_initial_pose_service_client.call_async(req)
        future.add_done_callback(self.handle_initial_pose_response)

    def handle_initial_pose_response(self, future):
        try:
            response = future.result()
            if response:
                self.get_logger().info("[INFO] Initial pose set successfully")
                self.initial_status = True
            else:
                self.get_logger().warn("[WARN] Failed to set initial pose")
        except Exception as e:
            self.get_logger().error(f"[ERROR] Service call failed: {e}")

    def run(self):
        rclpy.spin(self)

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
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
            self.get_logger().error(f"Invalid table_id: {table_id}")
            return
        
        # 목표 좌표 설정
        self.current_goal = table_id  # 현재 목표 설정
        target_pose = self.set_nav2_pose[table_id]
        
        # 목표 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_pose.x
        goal_msg.pose.pose.position.y = target_pose.y
        goal_msg.pose.pose.orientation.z = target_pose.z
        goal_msg.pose.pose.orientation.w = 1.0  # 방향 기본값

        # 서버 대기
        self._action_client.wait_for_server()
        self.get_logger().info(
            f"Sending goal to: x={target_pose.x}, y={target_pose.y}, theta={target_pose.z}"
        )
        
        # 목표 전송
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Current pose: {feedback.current_pose}")

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
            
            # 목표 지점에서 3초 대기 후 복귀 목표로 이동
            if self.current_goal != '0':  # 복귀 목표가 아니라면
                self.get_logger().info("Waiting for 3 seconds before returning...")
                time.sleep(3)
                self.get_logger().info("Switching to return goal (table_id='0').")
                self.send_goal('0')  # 복귀 지점으로 목표 전환
            else:
                self.get_logger().info("Return goal reached. Shutting down.")
                rclpy.shutdown()  # 복귀 목표까지 완료되면 종료
        else:
            self.get_logger().warn(f"Goal failed with status: {result.status}")
            rclpy.shutdown()



class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.set_nav2_pose = {
            '0': Point(x=0.0, y=0.0, z=0.0),  # 복귀 지점
            '1': Point(x=2.55, y=1.6, z=0.0),  # table_id = '1'
            '5': Point(x=1.43, y=0.47, z=0.0)   # table_id = '5'
        }

    def send_waypoints(self, table_ids):
        waypoints = []

        for table_id in table_ids:
            table_id_str = str(table_id)
            if table_id_str not in self.set_nav2_pose:
                self.get_logger().error(f"Invalid table_id: {table_id_str}")
                return
            
            target_pose = self.set_nav2_pose[table_id_str]
            waypoints.append((target_pose.x, target_pose.y, target_pose.z))

        # 복귀 지점 추가
        return_pose = self.set_nav2_pose['0']
        waypoints.append((return_pose.x, return_pose.y, return_pose.z))

        # NavigateThroughPoses 메시지 생성
        goal_msg = NavigateThroughPoses.Goal()
        timestamp = self.get_clock().now().to_msg()
        goal_msg.poses = []

        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = timestamp
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.orientation.z = wp[2]
            pose.pose.orientation.w = 1.0
            goal_msg.poses.append(pose)

        self._action_client.wait_for_server()
        self.get_logger().info(f"Sending {len(waypoints)} waypoints...")
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        
        self.get_logger().info(f"Current waypoint feedback: {feedback}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Waypoints rejected :(")
            return

        self.get_logger().info("Waypoints accepted. Navigating...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            result = future.result()
            if result.status == 4:  # STATUS_SUCCEEDED
                self.get_logger().info("Navigation completed successfully!")
            else:
                self.get_logger().warn(f"Navigation failed with status: {result.status}")
        except Exception as e:
            self.get_logger().error(f"Error during navigation: {e}")
        finally:
            self.get_logger().info("Node will shut down.")
            self.destroy_node()  # 노드 안전하게 종료
            rclpy.shutdown()


def main():
    rclpy.init()

    # InitialPose 노드를 실행하고 완료될 때까지 대기
    init_node = InitialPose()

    # InitialPose가 완료될 때까지 대기
    while not init_node.initial_status:
        rclpy.spin_once(init_node)

    time.sleep(3)
    # InitialPose 노드가 완료되었으면 GoalSender 노드를 실행
    goal_node = GoalSender()
    try:
        # 목표 전송: 예시로 table_id '3'을 전송
        goal_node.send_goal('3')  # '3'을 목표로 전송
        # 목표가 끝나면 자동으로 통신을 종료하도록 rclpy.spin() 대신 다음과 같이 작성
        rclpy.spin_once(goal_node)  # 목표를 수행하고, 이 노드가 완료될 때까지 대기
    except KeyboardInterrupt:
        goal_node.get_logger().info("KeyboardInterrupt, shutting down.")
    finally:
        # 목표가 완료되면 노드를 종료
        goal_node.get_logger().info("Goal has been reached, shutting down nodes.")
        goal_node.destroy_node()
        init_node.destroy_node()
        rclpy.shutdown()




if __name__ == '__main__':
    main()
