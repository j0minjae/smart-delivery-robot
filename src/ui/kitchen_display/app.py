import sys

from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QWidget, QListWidgetItem
from PyQt5.QtCore import pyqtSlot, QFile, QTextStream, Qt

from widgets.main_ui import Ui_MainWindow
from widgets.order_label_ui import Ui_order_label_container as Ui_Order_Label
from widgets.order_ticket_ui import Ui_Form as Ui_Order_Ticket
from widgets.table_info_ui import Ui_Form as Ui_Table_Info

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
    def __init__(self):
        super().__init__()
        self.order_ticket_ui = Ui_Order_Ticket()
        self.order_ticket_ui.setupUi(self)
        
        self.table_num = self.order_ticket_ui.table_num
        self.time_since_order = self.order_ticket_ui.time_since_order

    def set_table_num(self, table_num):
        self.table_num.setText(str(table_num))

    def set_time(self, time:str):
        self.time_since_order.setText(time)


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

        item1 = QListWidgetItem("Item 1")
        item2 = QListWidgetItem("Item 2")
        self.ui.order_tickets_list.addItem(item1)
        self.ui.order_tickets_list.addItem(item2)
        self.ui.order_tickets_list.setItemWidget(item1, self.create_order_ticket(9))

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
    
    def create_order_ticket(self, table_num:int, time:str="00:00"):
        order_ticket = OrderTicket()
        order_ticket.set_table_num(table_num)
        order_ticket.set_time(time)

        return order_ticket
    
    
if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())