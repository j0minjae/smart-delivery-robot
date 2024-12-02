import sys
from datetime import datetime

from DTO import TableInfo, OrderTicket, MenuItem
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QWidget, QLabel, QVBoxLayout
from PyQt5.QtCore import pyqtSlot, QFile, QTextStream, Qt, QTimer

from .data_management import DataManager, TableManager, TicketManager
from .widgets.main_ui import Ui_MainWindow
from .widgets.order_label_ui import Ui_order_label_container as Ui_Order_Label
from .widgets.order_ticket_ui import Ui_Form as Ui_Order_Ticket
from .widgets.table_info_ui import Ui_Form as Ui_Table_Info

from PyQt5.QtChart import QChart, QChartView, QBarSet, QBarSeries, QBarCategoryAxis, QValueAxis
from PyQt5.QtGui import QPainter

menu_item_map = {
    1: "히레카츠",
    2: "로스카츠",
    3: "고구마치즈돈까스",
    4: "치즈카츠",
    5: "왕돈까스",
    6: "모둠돈까스",
}

class SingleOrder:
    def __init__(self, menu_id, quantity):
        self.menu_id = menu_id
        self.quantity = quantity

class OrderLabel(QWidget):
    def __init__(self, menu:MenuItem, order_change_callback:callable):
        super().__init__()
        self.order_label_ui = Ui_Order_Label()
        self.order_label_ui.setupUi(self)
        self.description =  self.order_label_ui.description

        self.checkbox = self.order_label_ui.checkBox

        self.menu = menu
        self.order_change_callback = order_change_callback

        self.checkbox.toggled.connect(self.on_checkbox_toggled)
        
        self.update_widget()

    def set_description(self, description):
        self.description.setText(description)

    def update_widget(self):
        self.checkbox.setChecked(self.menu.is_checked)
        self.setDisabled(self.menu.is_disable)
        self.set_description(f"{menu_item_map[self.menu.menu_id]} X {self.menu.quantity}")

    def on_checkbox_toggled(self):
        self.menu.is_checked = self.checkbox.isChecked()
        self.order_change_callback()



class TableInfoWidget(QWidget):
    def __init__(self, data_manager:TableManager=None):
        super().__init__()
        self.data_manager = data_manager

        self.table_info_ui = Ui_Table_Info()
        self.table_info_ui.setupUi(self)

        self.data_manager.table_update.connect(self.update_widget)
        
        self.table_num = self.table_info_ui.table_num
        self.first_order_time = self.table_info_ui.first_order_time
        self.total_amount = self.table_info_ui.total_amount
        self.order_layout = self.table_info_ui.order_layout

        self.data_manager.update_table()

    def set_table_num(self, table_num):
        self.table_num.setText(str(table_num))

    def set_time(self, time:str):
        self.first_order_time.setText(time)
    
    def set_total(self, total:int):
        self.total_amount.setText(str(total))
    
    def set_order(self, order:list[MenuItem]):
        self.clear_order_layout()

        for menu in order:
            menu_label = QLabel(f"{menu_item_map[menu.menu_id]} X {menu.quantity}")
            self.order_layout.addWidget(menu_label)

    def update_widget(self):
        model = self.data_manager.model
        
        self.set_table_num(model.table_id)
        
        if model.arrival_time:
            self.set_time(model.arrival_time.strftime("%H:%M"))
        else:
            self.set_time("")

        self.set_total(model.payment)
        self.set_order(model.order)

    def clear_order_layout(self):
        for i in reversed(range(self.order_layout.count())):
            item = self.order_layout.itemAt(i)
            if item is not None:
                widget = item.widget()
                if widget is not None:
                    widget.deleteLater()

class OrderTicketWidget(QWidget):
    def __init__(self, data_manager:TicketManager=None):
        super().__init__()
        self.data_manager = data_manager

        self.order_ticket_ui = Ui_Order_Ticket()
        self.order_ticket_ui.setupUi(self)

        self.data_manager.ticket_update.connect(self.update_widget)
        self.data_manager.timer_update.connect(self.update_time)
        
        self.table_num = self.order_ticket_ui.table_num
        self.table_num = self.order_ticket_ui.table_num
        self.time_since_order = self.order_ticket_ui.time_since_order

        self.order_layout = self.order_ticket_ui.orders

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_callback)
        self.timer.start(1000)

        self.set_order(self.data_manager.model.order)
        self.update_widget()

    def update_time(self):
        time = self.data_manager.model.elapsed
        self.set_time(str(time))
    
    def timer_callback(self):
        self.data_manager.increase_time()

    def set_table_num(self, table_num):
        self.table_num.setText(str(table_num))

    def set_time(self, time:str):
        self.time_since_order.setText(time)

    def set_order(self, order:list[MenuItem]):
        self.clear_order_layout()

        for menu in order:
            menu_label = OrderLabel(menu, self.order_change_callback)
            self.order_layout.addWidget(menu_label, alignment=Qt.AlignTop)

    def update_order(self):
        for i in range(self.order_layout.count()):
            item = self.order_layout.itemAt(i)
            if item is not None:
                order_label = item.widget()
                if order_label and isinstance(order_label, OrderLabel):
                    order_label.update_widget()

    def update_widget(self):
        model = self.data_manager.model
        
        self.setDisabled(self.data_manager.is_disable)
        self.set_table_num(model.table_id)
        self.set_time(str(model.elapsed))
        self.update_order()

    def clear_order_layout(self):
        for i in reversed(range(self.order_layout.count())):
            item = self.order_layout.itemAt(i)
            if item is not None:
                widget = item.widget()
                if widget is not None:
                    widget.deleteLater()
    
    def order_change_callback(self):
        self.data_manager.check_order()

    def mousePressEvent(self, event):
        self.data_manager.check_all_order()
            

class MainWindow(QMainWindow):
    def __init__(self, data_manager:DataManager=None):
        super(MainWindow, self).__init__()

        #Data Manager
        self.data_manager = data_manager

        #Ui default settings
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        self.table_layout = self.ui.table_info_layout
        self.ticket_layout =  self.ui.order_tickets_layout
        self.log_browser = self.ui.order_page_log_browser
        self.week_chart_widget = self.ui.chart_a_layout
        self.month_chart_widget = self.ui.chart_b_layout
        self.menu_chart_widget = self.ui.chart_c_layout

        self.page = 0
        self.ui.stackedWidget.setCurrentIndex(self.page)
        

        self.data_manager.tables_update.connect(self.render_tables)
        self.data_manager.tickets_update.connect(self.render_tickets)
        self.data_manager.logs_update.connect(self.render_logs)
        self.data_manager.chart_update.connect(self.render_chart)

        self.ui.serve_btn.clicked.connect(self.on_serve_button_click)
        self.ui.call_btn.clicked.connect(self.on_call_button_click)
        self.ui.robot_serve_btn.clicked.connect(self.on_robot_serve_button_click)

        self.ui.order_page_btn.clicked.connect(self.on_order_page_button_click)
        self.ui.chart_page_btn.clicked.connect(self.on_chart_page_button_click)

        #data manager initial refresh
        self.data_manager.refresh_all()
    
    def render_tables(self):
        #create table_info list by table managers
        table_managers = self.data_manager.tables
        table_infos = [self.create_table_info(table_manager) for table_manager in table_managers]
        
        #Clear table layout and add new ones
        self.clear_table_layout()
        for i,table in enumerate(table_infos):
            row = i // 3
            column = i%3
            self.table_layout.addWidget(table,row,column)

    def clear_table_layout(self):
        for i in reversed(range(self.table_layout.count())):
            item = self.table_layout.itemAt(i)
            if item is not None:
                widget = item.widget()
                if widget is not None:
                    widget.deleteLater() 
    
    def render_tickets(self):
        ticket_managers = self.data_manager.tickets
        order_tickets = [self.create_order_ticket(ticket_manager) for ticket_manager in ticket_managers]
        
        self.clear_ticket_layout()
        for order_ticket in order_tickets:
            self.ticket_layout.addWidget(order_ticket, alignment=Qt.AlignTop)

    def render_logs(self):
        logs = self.data_manager.logs

        log_level_map = {10:"DEBUG", 20:"INFO", 30:"WARN", 40:"ERROR", 50:"FATAL"}
        
        self.log_browser.clear()
        for log in logs:
            self.log_browser.append(f"[{log.name}][{log_level_map[log.level]}]{log.msg}")

    def render_chart(self):
        data = self.data_manager.chart_data
        if data == None:
            self.data_manager.emit_log("no chart data",level=30)
            return
        
        week_data, month_data, menu_data = data['week'], data['month'], data['menu']
        
        menu_data = [(menu_item_map[item[0]], item[1]) for item in menu_data]
        
        week = ["일","월", "화", "수","목","금","토"]
        week_data = [(week[i], item[1]) for i, item in enumerate(week_data)]

        week_chart = self.make_chart("week",week_data)
        month_chart = self.make_chart("month",month_data)
        menu_chart = self.make_chart("menu",menu_data)

        self.week_chart_widget.addWidget(week_chart)
        self.month_chart_widget.addWidget(month_chart)
        self.menu_chart_widget.addWidget(menu_chart)

    def make_chart(self, name, data):
        set0 = QBarSet(name)
        for item in data:
            set0.append(item[1])
        
        series = QBarSeries()
        series.append(set0)

        chart = QChart()
        chart.addSeries(series)
        chart.setTitle(name)
        chart.setAnimationOptions(QChart.SeriesAnimations)

        categories = [str(item[0]) for item in data]
        axisX = QBarCategoryAxis()
        axisX.append(categories)
        chart.addAxis(axisX, Qt.AlignBottom)
        series.attachAxis(axisX)

        # Y축 설정 (값 축)
        axisY = QValueAxis()
        max_y = max([item[1] for item in data]) + 100000
        axisY.setRange(0, max_y)  # Y축 범위 설정
        chart.addAxis(axisY, Qt.AlignLeft)
        series.attachAxis(axisY)

        # 차트 뷰 생성
        chart_view = QChartView(chart)
        chart_view.setRenderHint(QPainter.Antialiasing)

        # layout = QVBoxLayout()
        # layout.addWidget(chart_view)

        return chart_view
        


    def clear_ticket_layout(self):
        for i in reversed(range(self.ticket_layout.count())):
            item = self.ticket_layout.itemAt(i)
            if item is not None:
                widget = item.widget()
                if widget is not None:
                    widget.deleteLater() 



    def create_table_info(self, table_manager:TableManager):
        table_info = TableInfoWidget(table_manager)
        return table_info
    
    def create_order_ticket(self, ticket_manager:TicketManager):
        order_ticket = OrderTicketWidget(ticket_manager)
        return order_ticket

    def on_serve_button_click(self):
        self.data_manager.serve_checked_menu()
    
    def on_robot_serve_button_click(self):
        self.data_manager.serve_checked_menu_by_robot()

    def on_call_button_click(self):
        self.data_manager.call_robot()
    
    def on_order_page_button_click(self):
        self.page = 0
        self.ui.stackedWidget.setCurrentIndex(self.page)

    def on_chart_page_button_click(self):
        self.data_manager.emit_chart_signal()
        self.page = 1
        self.ui.stackedWidget.setCurrentIndex(self.page)
         
    
    
if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())