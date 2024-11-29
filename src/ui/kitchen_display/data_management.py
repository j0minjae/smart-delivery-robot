import copy
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
    
    def update_table(self, table:TableInfo=None):
        if table:
            self.model = table
        self.table_update.emit()

class TicketManager(QObject):
    ticket_update = pyqtSignal()

    def __init__(self, ticket:OrderTicket=None):
        super().__init__()

        self.model = ticket

    def set_order(self, order):
        self.model.order = order
        self.ticket_update.emit()
    
    def update_ticket(self):
        self.ticket_update.emit()

class DataManager(QObject):
    tables_update = pyqtSignal()
    tickets_update = pyqtSignal()

    def __init__(self, tables:list[TableInfo]=list(), tickets:list[OrderTicket]=list()):
        super().__init__()
        
        self.tables = [TableManager(table) for table in tables]
        self.tickets = [TicketManager(ticket) for ticket in tickets]

    def refresh_all(self):
        self.refresh_tables()
        self.refresh_tickets()

    def refresh_tables(self):
        self.tables_update.emit()
    
    def refresh_tickets(self):
        self.tickets_update.emit()
    
    def create_ticket(self, ticket: OrderTicket):
        self.tickets.insert(0, TicketManager(ticket))
        self.refresh_tickets()

    def create_order(self,ticket:OrderTicket):
        self.create_ticket(ticket)
        
        for table_manager in self.tables:
            if table_manager.model.table_id == ticket.table_id:
                new_ticket = copy.deepcopy(table_manager.model)
                new_ticket.order.extend(ticket.order)
                table_manager.update_table(new_ticket)


