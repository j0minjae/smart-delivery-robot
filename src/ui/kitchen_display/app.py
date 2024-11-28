import sys

from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QWidget
from PyQt5.QtCore import pyqtSlot, QFile, QTextStream, Qt

from widgets.main_ui import Ui_MainWindow
from widgets.order_label_ui import Ui_order_label_container as Ui_Order_Label
from widgets.order_ticket_ui import Ui_Form as Ui_Order_Ticket
from widgets.table_info_ui import Ui_Form as Ui_Table_Info

class SingleOrder:
    def __init__(self, menu_id, quantity):
        self.menu_id = menu_id
        self.quantity = quantity

class OrderLabel(QWidget):
    def __init__(self, menu_id, quantity):
        super().__init__()
        self.order_label_ui = Ui_Order_Label()
        self.order_label_ui.setupUi(self)
        self.description =  self.order_label_ui.description   
        
        self.menu_id = menu_id
        self.quantity = quantity
        
        self.set_description(f"{quantity} X {menu_id}")

    def set_description(self, description):
        self.description.setText(description)

class TableInfoWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.table_info_ui = Ui_Table_Info()
        self.table_info_ui.setupUi(self)
        
        self.table_num = self.table_info_ui.table_num
        self.first_order_time = self.table_info_ui.first_order_time
        self.total_amount = self.table_info_ui.total_amount

    def set_table_num(self, table_num):
        self.table_num.setText(str(table_num))

    def set_time(self, time:str):
        self.first_order_time.setText(time)
    
    def set_total(self, total:int):
        self.total_amount.setText(str(total))
        

class OrderTicket(QWidget):
    def __init__(self, data:list[SingleOrder]):
        super().__init__()
        self.order_ticket_ui = Ui_Order_Ticket()
        self.order_ticket_ui.setupUi(self)
        
        self.table_num = self.order_ticket_ui.table_num
        self.time_since_order = self.order_ticket_ui.time_since_order
        self.order_list = self.order_ticket_ui.orders

        self.set_order_list_by_data(data)
        

    def set_table_num(self, table_num):
        self.table_num.setText(str(table_num))

    def set_time(self, time:str):
        self.time_since_order.setText(time)
    
    def set_order_list(self, orders:list[OrderLabel]):
        for order in orders:
            self.order_list.addWidget(order, alignment=Qt.AlignTop)

    def set_order_list_by_data(self, orders:list[SingleOrder]):
        """set order list by orders

        Args:
            orders (list[SingleOrder]): list of SingleOrder
        """
        order_list = []
        for single_order in orders:
            order_label = OrderLabel(single_order.menu_id, single_order.quantity)
            order_list.append(order_label)
        
        self.set_order_list(order_list)
            

class MainWindow(QMainWindow):
    def __init__(self, table_list = [1,2,3,4,5,6,7,8,9]):
        super(MainWindow, self).__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.stackedWidget.setCurrentIndex(0)
        
        for i,table in enumerate(self.create_tables(table_list)):
            row = i // 3
            column = i%3
            self.ui.table_info_layout.addWidget(table,row,column)

        self.selected_table = None
        self.ui.serve_btn.clicked.connect(lambda: self.on_serve_button_click(self.selected_table))
        self.ui.call_btn.clicked.connect(self.on_call_button_click)

    def create_tables(self, table_list):
        return [self.create_table_info(table) for table in table_list]
    
    def create_table_info(self, table_num:int, first_order_time:str = "", total_amount:int=None):
        """Create New Table Info

        Args:
            table_num (int): table number
            first_order_time (str, optional): time of the first table order. Defaults to "".
            total_amount (int, optional): total amount of the table. Defaults to None.

        Returns:
            TableInfoWidget: single table info widget
        """
        table_info = TableInfoWidget()
        table_info.set_table_num(table_num)
        table_info.set_time(first_order_time)
        table_info.set_total(total_amount)

        return table_info
    
    def create_order_ticket(self, table_num:int=1, time:str="00:00", order_data:list[SingleOrder]=list()):
        order_ticket = OrderTicket(order_data)
        order_ticket.set_table_num(table_num)
        order_ticket.set_time(time)

        return order_ticket
    
    def create_order(self, order:dict):
        """_summary_

        Args:
            order (dict): {"table_id":int, "orders":[{"menu_id":int, "quantity":int}]}

        Returns:
            _type_: _description_
        """
        orders = []
        table_id = order["table_id"]
        orders_data = order["orders"]
        for order_data in orders_data:
            menu_id = order_data["menu_id"]
            quantity = order_data["quantity"]
            orders.append(SingleOrder(menu_id, quantity))
        
        order_ticket = self.create_order_ticket(table_id,"00:00",orders)
        
        return self.ui.order_tickets_layout.addWidget(order_ticket, alignment=Qt.AlignTop)
    
    def on_serve_button_click(self, table_id):
        if table_id:
            print(f"{table_id}로 로봇을 보냅니다")
        else:
            print("보낼 테이블이 정해지지 않았습니다")

    def on_call_button_click(self):
        print("로봇을 호출합니다")
         
    
    
if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = MainWindow()

    # order = {"table_id":1, "orders":[{"menu_id":1, "quantity":2},{"menu_id":2, "quantity":1}]}

    # window.create_order(order)
    window.show()

    sys.exit(app.exec())