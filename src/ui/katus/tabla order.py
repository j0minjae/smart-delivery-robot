import sys
from datetime import datetime
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QSpinBox, QScrollArea, QFrame, QDialog, QMessageBox
)

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
    def __init__(self):
        super().__init__()

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
        self.spin_boxes = {}  # QSpinBox 객체를 관리할 딕셔너리

        self.init_ui()

    def init_ui(self):
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        main_layout = QHBoxLayout(self.central_widget)

        # 메뉴 영역
        self.menu_area = self.create_menu_area()

        # 장바구니 영역
        self.cart_area = self.create_cart_area()

        main_layout.addWidget(self.menu_area, stretch=3)
        main_layout.addWidget(self.cart_area, stretch=1)

    def create_menu_area(self):
        menu_area = QWidget()
        layout = QVBoxLayout(menu_area)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setSpacing(20)

        for item in self.menu_items:
            card = self.create_menu_card(item)
            scroll_layout.addWidget(card)

        scroll_area.setWidget(scroll_content)
        layout.addWidget(scroll_area)

        return menu_area

    def create_menu_card(self, item):
        card = QFrame()
        card.setStyleSheet("""
            QFrame {
                background-color: #2C2C2C;
                border: 2px solid #FFD700;
                border-radius: 15px;
                padding: 15px;
                margin: 10px;
                box-shadow: 2px 2px 5px rgba(0, 0, 0, 0.5);
            }
        """)
        layout = QHBoxLayout(card)

        pixmap = QPixmap(item["image"]).scaled(400, 800, Qt.KeepAspectRatio)
        image_label = QLabel()
        image_label.setPixmap(pixmap)
        image_label.setStyleSheet("border: 3px solid #FFD700; border-radius: 10px;")
        layout.addWidget(image_label)

        details_layout = QVBoxLayout()
        name_label = QLabel(item["name"])
        name_label.setFont(QFont("Georgia", 20, QFont.Bold))
        name_label.setStyleSheet("color: #F7F1E3; margin-bottom: 5px;")

        price_label = QLabel(f"{item['price']}원")
        price_label.setFont(QFont("Georgia", 16))
        price_label.setStyleSheet("color: #F5C35A;")

        spin_box = QSpinBox()
        spin_box.setRange(1, 10)
        spin_box.setFixedSize(80, 40)
        spin_box.setStyleSheet("""
            QSpinBox {
                background-color: #FFFFFF;
                border: 2px solid #FFD700;
                border-radius: 5px;
                padding: 5px;
                font-size: 16px;
            }
        """)

        # 메뉴 이름을 키로 QSpinBox 저장
        self.spin_boxes[item["name"]] = spin_box

        add_button = QPushButton("추가")
        add_button.setStyleSheet("""
            QPushButton {
                background-color: #FFD700;
                color: #2C2C2C;
                border-radius: 5px;
                font-size: 16px;
                font-weight: bold;
                padding: 5px 10px;
            }
        """)
        add_button.clicked.connect(lambda: self.add_to_cart(item, spin_box.value()))

        details_layout.addWidget(name_label)
        details_layout.addWidget(price_label)
        details_layout.addWidget(spin_box)
        details_layout.addWidget(add_button)

        layout.addLayout(details_layout)
        return card

    def create_cart_area(self):
        cart_area = QFrame()
        cart_area.setStyleSheet("""
            QFrame {
                background-color: #2C2C2C;
                border: 2px solid #FFD700;
                border-radius: 10px;
                padding: 10px;
            }
        """)

        layout = QVBoxLayout(cart_area)

        cart_title = QLabel("장바구니")
        cart_title.setFont(QFont("Georgia", 16, QFont.Bold))
        cart_title.setStyleSheet("color: #FFD700; margin-bottom: 5px;")
        cart_title.setAlignment(Qt.AlignCenter)
        layout.addWidget(cart_title)

        self.cart_list = QVBoxLayout()

        cart_scroll_area = QScrollArea()
        cart_scroll_area.setFixedHeight(600)
        cart_scroll_area.setWidgetResizable(True)
        cart_scroll_content = QWidget()
        cart_scroll_content.setLayout(self.cart_list)
        cart_scroll_area.setWidget(cart_scroll_content)

        layout.addWidget(cart_scroll_area)

        order_button = QPushButton("주문 완료")
        order_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: #FFFFFF;
                border-radius: 5px;
                font-size: 16px;
                font-weight: bold;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #45A049;
            }
        """)
        order_button.clicked.connect(self.confirm_order)
        layout.addWidget(order_button)

        history_button = QPushButton("주문 내역 보기")
        history_button.setStyleSheet("""
            QPushButton {
                background-color: #1E90FF;
                color: #FFFFFF;
                border-radius: 5px;
                font-size: 16px;
                font-weight: bold;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #1C86EE;
            }
        """)
        history_button.clicked.connect(self.show_order_history)
        layout.addWidget(history_button)

        return cart_area

    def add_to_cart(self, item, quantity):
        if item["name"] in self.cart:
            self.cart[item["name"]]["quantity"] += quantity
        else:
            self.cart[item["name"]] = {"price": item["price"], "quantity": quantity}
        self.update_cart_view()

    def update_cart_view(self):
        while self.cart_list.count():
            child = self.cart_list.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

        for name, details in self.cart.items():
            cart_item_widget = self.create_cart_item_widget(name, details)
            self.cart_list.addWidget(cart_item_widget)

    def create_cart_item_widget(self, name, details):
        widget = QFrame()
        widget.setStyleSheet("""
            QFrame {
                background-color: #2C2C2C;
                border: 2px solid #FFD700;
                border-radius: 10px;
                padding: 10px;
                margin-bottom: 5px;
                width: 200px;
                height: 100px;
            }
        """)

        layout = QVBoxLayout(widget)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(3)

        name_label = QLabel(name)
        name_label.setFont(QFont("Georgia", 12, QFont.Bold))
        name_label.setStyleSheet("color: #FFD700;")
        layout.addWidget(name_label)

        quantity_label = QLabel(f"수량: {details['quantity']}개")
        quantity_label.setFont(QFont("Georgia", 10))
        quantity_label.setStyleSheet("color: #FFFFFF;")
        layout.addWidget(quantity_label)

        price_label = QLabel(f"가격: {details['quantity'] * details['price']}원")
        price_label.setFont(QFont("Georgia", 10))
        price_label.setStyleSheet("color: #F5C35A;")
        layout.addWidget(price_label)

        remove_button = QPushButton("삭제")
        remove_button.setStyleSheet("""
            QPushButton {
                background-color: #FF6B6B;
                color: #FFFFFF;
                border-radius: 5px;
                font-size: 10px;
                font-weight: bold;
                padding: 3px 5px;
            }
            QPushButton:hover {
                background-color: #FF4C4C;
            }
        """)
        remove_button.clicked.connect(lambda: self.remove_cart_item(name))
        layout.addWidget(remove_button)

        return widget

    def remove_cart_item(self, name):
        if name in self.cart:
            del self.cart[name]
        self.update_cart_view()

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

        # 장바구니 초기화
        self.cart.clear()
        self.update_cart_view()

        # 모든 QSpinBox 초기화
        for spin_box in self.spin_boxes.values():
            spin_box.setValue(1)

    def show_order_history(self):
        try:
            with open("orders.txt", "r", encoding="utf-8") as file:
                history = file.readlines()
        except FileNotFoundError:
            history = ["주문 기록이 없습니다."]

        dialog = OrderDialog("주문 내역 보기", history, total_price=0)
        dialog.exec_()

def main():
    app = QApplication(sys.argv)
    window = CafeMenuApp()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
