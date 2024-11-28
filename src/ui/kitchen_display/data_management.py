from DTO import MenuItem, OrderTicket, TableInfo
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QWidget
from PyQt5.QtCore import QFile, QTextStream, Qt, QObject, pyqtSignal


class TableManager(QObject):
    table_update = pyqtSignal()
    
    def __init__(self, table:TableInfo=None):
        super().__init__()

        self.model = table

    def set_status(self, status:bool):
        if self.model.status != status:
            self.model.status = status
            self.table_update.emit()

    def set_order(self, order):
        if self.model.order != order:
            self.model.order = order
            self.table_update.emit()
    
    def update_table(self):
        self.table_update.emit()

class DataManager(QObject):
    tables_update = pyqtSignal()
    tickets_update = pyqtSignal()

    def __init__(self, tables:list[TableInfo]=list(), tickets:list[OrderTicket]=list()):
        super().__init__()
        
        self.tables = [TableManager(table) for table in tables]
        self.tickets = tickets

        # for table_manager in self.tables:
        #     self.tables_update.connect(table_manager.update_table)

    def refresh_all(self):
        self.refresh_tables()
        self.refresh_tickets()

    def refresh_tables(self):
        self.tables_update.emit()
    
    def refresh_tickets(self):
        self.tickets_update.emit()


