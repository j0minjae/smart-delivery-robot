from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QPushButton

class OrderUI(QWidget):
    def __init__(self):
        super().__init__()

        # UI 요소 정의
        self.label = QLabel("Order Data: Not received", self)
        self.button = QPushButton("Update Order", self)

        # 레이아웃 설정
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.button)

        self.setLayout(layout)
        
        # 버튼 클릭 시 동작 연결
        self.button.clicked.connect(self.update_order_data)

    def update_order_data(self, table_id, order):
        # 여기에 'order' 노드에서 데이터를 받아오는 로직을 작성
        # 예시: self.receive_data_from_order({"order_id": 123, "status": "Completed"})
        order_data = {"table number": {table_id}, "menu": {order}}
        self.receive_data_from_order(order_data)

    def receive_data_from_order(self, order_data):
        """ order 데이터 처리 및 UI 업데이트 """
        order_info = f"table number: {order_data['table number']}, menu: {order_data['menu']}"
        self.label.setText(order_info)
        print(f"Received order data: {order_data}")

    def update_log(self, log):
        return print(log)

def main():
    app = QApplication([])
    ui = OrderUI()
    ui.show()
    app.exec_()

if __name__ == '__main__':
    main()