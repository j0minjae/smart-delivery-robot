import rclpy
import sys
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication
from db_management import OrderManager
from ssts.ssts import gui
from ui import MainWindow

class ROS2Thread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        # ROS2 spin을 별도의 스레드에서 실행
        rclpy.spin(self.node)

    def stop(self):
        # 스레드를 종료할 때 spin 종료
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    main_window = MainWindow()

    db_instance = OrderManager()
    
    node = gui(db_instance)

    #ros2 thread 실행
    ros2_thread = ROS2Thread(node)
    ros2_thread.start()

    #ui표시
    main_window.show()

    sys.exit(app.exec_())

    ros2_thread.stop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()