import sys
import os
import time
import random
from datetime import datetime
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QApplication

from ui import MainWindow, DataManager, TableManager
from DTO import TableInfo, OrderTicket, MenuItem

class TestTimerThread(QThread):
    def __init__(self, data_manager: DataManager):
        super().__init__()
        self.data_manager = data_manager
        self.interval = 10
        self.running = True

    def run(self):
        time.sleep(5)
        while self.running:
            self.some_function()
            time.sleep(self.interval)  # 간격만큼 대기

    def some_function(self):
        table_id = random.randint(1,9)
        orders = [[1,2],[2,1],[3,1]]
        orders = [MenuItem(menu[0], menu[1]) for menu in orders]
        ticket = OrderTicket(table_id, order=orders)
        self.data_manager.create_order(ticket)

    def stop(self):
        self.running = False

def main(args=None):
    table_infos = [TableInfo(table_id=i) for i in range(1,10)]
    order_tickets = [OrderTicket(table_id=i, order=[
        MenuItem(1,2,False),
    ]) for i in range(1,4)]

    data_manager = DataManager(tables=table_infos)

    app = QApplication(sys.argv)
    main_window = MainWindow(data_manager)

    tester = TestTimerThread(data_manager=data_manager)
    tester.start()

    #ui표시
    main_window.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()