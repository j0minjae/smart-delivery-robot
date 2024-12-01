import rclpy
from rclpy.duration import Duration
import sys
from datetime import datetime
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication
from db_management import OrderManager
from ssts import gui
from control_manager import InitialPose
from ui import MainWindow, DataManager, TableManager
from DTO import TableInfo, OrderTicket, MenuItem

class ROS2Thread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)

    def stop(self):
        # 스레드를 종료할 때 spin 종료
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    table_infos = [TableInfo(table_id=i) for i in range(1,10)]

    data_manager = DataManager(tables=table_infos)

    app = QApplication(sys.argv)
    main_window = MainWindow(data_manager)

    db_instance = OrderManager()
    
    node = gui(db_instance, data_manager=data_manager, enable_control_manager=False)

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