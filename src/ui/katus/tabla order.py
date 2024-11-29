import sys
from datetime import datetime
from PyQt5.QtCore import Qt, QThread
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QSpinBox, QScrollArea, QFrame, QDialog, QMessageBox
)
import rclpy
from rclpy.node import Node
from serving_bot_interfaces.srv import PlaceOrder  # ROS 2 서비스 인터페이스 추가

class OrderDialog(QDialog):
    def __init__(self, title, details, total_price, confirm=False, parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setGeometry(200, 200, 400, 400)
        self.setStyleSheet("""
            QDialog {
                background-color: #FFFFFF;
                border: 2px solid #FFD700;
                border-radius: 10px;
            }
        """)

        layout = QVBoxLayout(self)

        dialog_title = QLabel(title)
        dialog_title.setFont(QFont("Georgia", 20, QFont.Bold))
        dialog_title.setStyleSheet("color: #000000; margin-bottom: 10px;")
        dialog_title.setAlignment(Qt.AlignCenter)
        layout.addWidget(dialog_title)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)

        for detail in details:
            item_label = QLabel(detail)
            item_label.setFont(QFont("Georgia", 14))
            item_label.setStyleSheet("color: #000000; margin-bottom: 5px;")
            scroll_layout.addWidget(item_label)

        scroll_area.setWidget(scroll_content)
        layout.addWidget(scroll_area)

        total_label = QLabel(f"총 금액: {total_price}원")
        total_label.setFont(QFont("Georgia", 16, QFont.Bold))
        total_label.setStyleSheet("color: #000000; margin-top: 10px;")
        total_label.setAlignment(Qt.AlignRight)
        layout.addWidget(total_label)

        button_layout = QHBoxLayout()

        if confirm:
            confirm_button = QPushButton("확정")
            confirm_button.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: #FFFFFF;
                    border-radius: 5px;
                    font-size: 16px;
                    font-weight: bold;
                    padding: 5px 10px;
                }
                QPushButton:hover {
                    background-color: #45A049;
                }
            """)
            confirm_button.clicked.connect(self.accept)
            button_layout.addWidget(confirm_button)

        close_button = QPushButton("닫기")
        close_button.setStyleSheet("""
            QPushButton {
                background-color: #FFD700;
                color: #2C2C2C;
                border-radius: 5px;
                font-size: 16px;
                font-weight: bold;
                padding: 5px 10px;
            }
            QPushButton:hover {
                background-color: #F5C35A;
            }
        """)
        close_button.clicked.connect(self.reject)
        button_layout.addWidget(close_button)

        layout.addLayout(button_layout)


class CafeMenuApp(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node  # ROS 2 노드 추가
        self.setWindowTitle("두돈")
        self.setGeometry(100, 100, 1400, 900)

        self.menu_items = [
            {"name": "히레카츠 정식", "price": 13000, "image": "hirekatsu.jpeg"},
            {"name": "로스카츠 정식", "price": 12000, "image": "roskatsu.jpeg"},
            {"name": "고구마 치즈 돈까스", "price": 14000, "image": "sweetpotato_cheese.jpeg"},
            {"name": "치즈 돈까스", "price": 13000, "image": "cheese_katsu.jpeg"},
            {"name": "왕 돈까스", "price": 9000, "image": "king_katsu.jpeg"},
            {"name": "모둠 카츠", "price": 15000, "image": "mixed_katsu.jpeg"},
        ]

        self.cart = {}
        self.spin_boxes = {}
        self.init_ui()

    def init_ui(self):
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        main_layout = QHBoxLayout(self.central_widget)

        self.menu_area = self.create_menu_area()
        self.cart_area = self.create_cart_area()

        main_layout.addWidget(self.menu_area, stretch=3)
        main_layout.addWidget(self.cart_area, stretch=1)

    def confirm_order(self):
        if not self.cart:
            QMessageBox.warning(self, "주문 실패", "장바구니가 비어 있습니다.")
            return

        total_price = sum(details["price"] * details["quantity"] for details in self.cart.values())
        order_details = [
            f"{name} x {details['quantity']}개 - {details['quantity'] * details['price']}원"
            for name, details in self.cart.items()
        ]

        dialog = OrderDialog("주문 확인", order_details, total_price, confirm=True)
        if dialog.exec_() == QDialog.Accepted:
            self.finalize_order(total_price, order_details)

    def finalize_order(self, total_price, order_details):
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open("orders.txt", "a", encoding="utf-8") as file:
            file.write(f"--- 주문 시간: {now} ---\n")
            file.write("\n".join(order_details))
            file.write(f"\n총 금액: {total_price}원\n\n")

        dialog = OrderDialog("주문 완료", order_details, total_price)
        dialog.exec_()

        order_text = "\n".join(order_details)
        future = self.ros_node.send_order(order_text)

        def process_response():
            rclpy.spin_until_future_complete(self.ros_node, future)
            if future.done():
                try:
                    response = future.result()
                    QMessageBox.information(self, "응답", response.message if response.success else "주문 실패")
                except Exception as e:
                    QMessageBox.critical(self, "오류", str(e))

        process_response()

        self.cart.clear()
        self.update_cart_view()
        for spin_box in self.spin_boxes.values():
            spin_box.setValue(1)


def main():
    rclpy.init()
    ros_node = ROS2Node()
    ros_thread = QThread(target=lambda: rclpy.spin(ros_node), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    main_window = CafeMenuApp(ros_node)
    main_window.show()
    sys.exit(app.exec_())

    ros_thread.stop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
