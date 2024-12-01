import sqlite3
from datetime import datetime, timedelta
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
from ament_index_python.packages import get_package_share_directory
import os

class OrderManager:
    # 테이블: 생성
    # def __init__(self):
        # 데이터베이스 파일 경로 생성
        # db_path = '/home/pgt/doosan/serving-bot/database/database.db'
        # SQLite DB 연결
        
    def get_resource_file_path(self):
        # 공유 디렉토리 경로 얻기
        share_dir = get_package_share_directory('ssts')
        
        # 해당 파일 경로 생성
        resource_path = os.path.join(share_dir, 'database.db')
        print(resource_path)
        return resource_path

    # 테이블: 메뉴 구성
    def update_menu(self, input_menu):
        db_path = self.get_resource_file_path()
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        """
        입력 데이터를 기반으로 데이터베이스를 업데이트
        menu_id는 첫 메뉴부터 1로 순차적 부여
        ex)
        <input>
        input_menu = {
            'Sirloin cutlet': 10000,
            'Tenderloin cutlet': 11000,
            'Assorted Katsu': 16000,
            'Cheese Katsu': 13000
        }

        <output>
        menu_id  menu_name  menu_price
        -------  ---------  ----------
        1        로스카츠       10000     
        2        히레카츠       11000     
        3        모둠카츠       16000     
        4        치즈카츠       13000  
        """

        print('##Excute(update_menu)##')
        # 테이블 생성
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS menu (
                menu_id INTEGER PRIMARY KEY AUTOINCREMENT,
                menu_name TEXT UNIQUE,
                menu_price INTEGER
            );
        ''')
        conn.commit()
        print("- Table(menu) created (if it did not exist).")

        # 기존 데이터 초기화
        cursor.execute('DELETE FROM menu;')  # 테이블 비우기
        cursor.execute('DELETE FROM sqlite_sequence WHERE name = "menu";')  # AUTOINCREMENT 리셋
        conn.commit()
        print("- Table(menu) cleared and menu_id reset.")

        # 입력 데이터 삽입
        for menu_name, menu_price in input_menu.items():
            cursor.execute('''
                INSERT INTO menu (menu_name, menu_price)
                VALUES (?, ?);
            ''', (menu_name, menu_price))
            print(f"- Inserted: {menu_name} -> {menu_price}")

        # 변경 사항 커밋
        conn.commit()
        conn.close()
        print("- Database(menu) updated with input menu.")

    def update_db(self, table_id, orders):
        self.insert_order(table_id, orders)

    # 테이블: 주문수락 데이터 추출 및 삽입
    def insert_order(self, table_id, orders):
        try:
            db_path = self.get_resource_file_path()
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()
            """
            주문이 수락되면 database에 추가되도록 처리하며,
            menu 테이블이 비어 있는 경우 메시지를 출력합니다.
            ex)
            <input>
            place_order = {
                "table_id": 5,
                "orders": [
                    {"menu_id": "1", "quantity": 2},
                    {"menu_id": "2", "quantity": 2},
                    {"menu_id": "3", "quantity": 2}
                ]
            }

            <result>
            order_id  time_stamp           table_id  menu_id  menu_name  total_price
            --------  -------------------  --------  -------  ---------  -----------
            1         2024-11-28 12:21:24  5         1        로스카츠       20,000     
            2         2024-11-28 12:21:24  5         2        히레카츠       22,000     
            3         2024-11-28 12:21:24  5         3        모둠카츠       32,000     
            4         2024-11-28 12:23:34  5         1        로스카츠       20,000     

            """
            # SQLite DB 연결
            print('## Execute(insert_order) ##')

            # orders 테이블 생성 (menu_name 열 추가)
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS orders (
                    order_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    time_stamp TEXT,
                    table_id INTEGER,
                    menu_id INTEGER,
                    menu_name TEXT,
                    total_price INTEGER
                );
            ''')
            conn.commit()
            print("- Table(orders) created (if it did not exist).")

            # 메뉴 테이블 확인
            cursor.execute('SELECT COUNT(*) FROM menu;')
            menu_count = cursor.fetchone()[0]  # 메뉴 데이터 개수 확인
        except:
            self.get_logger().info(f'DB 시스템에 문제가 발생했습니다')    #send ui

        if menu_count == 0:
            print("Menu is empty. No orders can be processed.")
            conn.close()
            print("Database(menu) connection closed.")
            return

        for order in orders:
            menu_id = order.menu_id
            quantity = order.quantity

            # menu 테이블에서 menu_id에 맞는 이름과 가격 조회
            cursor.execute('SELECT menu_name, menu_price FROM menu WHERE menu_id = ?;', (menu_id,))
            result = cursor.fetchone()

            if result:
                menu_name, menu_price = result  # 이름과 가격 가져오기
                price = menu_price * quantity
                time_stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

                # orders 테이블에 주문 데이터 삽입
                cursor.execute('''
                    INSERT INTO orders (time_stamp, table_id, menu_id, menu_name, total_price)
                    VALUES (?, ?, ?, ?, ?);
                ''', (time_stamp, table_id, menu_id, menu_name, price))
                print(f"Order inserted: Menu ID {menu_id}, Name {menu_name}, Quantity {quantity}, Total Price {price}")
            else:
                print(f"Error: Menu ID {menu_id} not found in the menu database.")

        # 커밋 및 연결 종료
        conn.commit()
        conn.close()
        print("- Order saved to database.")
    
    def update_log(self, msg):
        node_name = msg.name
        log_level = msg.level
        message = msg.msg
        self.insert_log(node_name, log_level, message)

    def call_log(self):
        log_dic = {}
        log_lst = []
        conn = self.get_db_connection()
        logs = conn.execute('SELECT * FROM logs ORDER BY log_id DESC').fetchall()
        conn.close()
        for log in logs:
            log_dic['log_id'] = log[0]
            log_dic['timestamp'] = log[1]
            log_dic['node_name'] = log[2]
            log_dic['log_level'] = log[3]
            log_dic['message'] = log[4]
            log_lst.append(log_dic)
        print(log_lst)
        return log_lst
    
    def get_db_connection(self):
        db_path = self.get_resource_file_path()
        conn = sqlite3.connect(db_path)
        conn.row_factory = sqlite3.Row  # 각 행을 딕셔너리 형태로 반환
        return conn

    # 테이블: 로그 데이터 추출 및 삽입
    def insert_log(self, node_name, log_level, message):
        db_path = self.get_resource_file_path()
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        # 테이블 생성
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS logs (
                log_id INTEGER PRIMARY KEY AUTOINCREMENT,
                time_stamp TEXT,
                node_name TEXT,
                log_level TEXT,
                message TEXT
            );
        ''')
        conn.commit()
        print("Logs table created (if it did not exist).")

        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        cursor.execute('''
            INSERT INTO logs (time_stamp, node_name, log_level, message)
            VALUES (?, ?, ?, ?);
        ''', (timestamp, node_name, log_level, message))
        conn.commit()
        conn.close()
        print(f"Log saved: {timestamp, node_name, log_level, message}")

    # 통계: 지난 한 달간의 일일 매출을 시각화(꺾은선 그래프)
    # 필요에 따라 n일간의 일일 매출로 변경 가능
    def generate_sales_graph(self, days=30):
        db_path = self.get_resource_file_path()
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        # 오늘 날짜 기준으로 n일 전 날짜 계산
        start_date = datetime.now() - timedelta(days=days)
        start_date_str = start_date.strftime('%Y-%m-%d')

        # n일 간의 매출 합산: 날짜별로 총 매출 계산
        cursor.execute('''
            SELECT strftime('%Y-%m-%d', time_stamp) AS date, 
                SUM(CAST(REPLACE(total_price, ',', '') AS INTEGER)) AS daily_sales
            FROM orders
            WHERE time_stamp >= ?
            GROUP BY date
            ORDER BY date;
        ''', (start_date_str,))

        sales_data = cursor.fetchall()

        # 데이터가 있을 경우 꺾은선 그래프 생성
        if sales_data:
            # pandas DataFrame으로 변환
            df = pd.DataFrame(sales_data, columns=['Date', 'Daily Sales'])
            df['Date'] = pd.to_datetime(df['Date'])  # 날짜 형식 변환
            df['Daily Sales (KRW in 10k)'] = df['Daily Sales'] / 10000  # 만 원 단위로 변환

            # seaborn을 이용한 꺾은선 그래프
            plt.figure(figsize=(10, 6))
            ax = sns.lineplot(x='Date', y='Daily Sales (KRW in 10k)', data=df, marker='o', color='b')

            # 각 데이터 포인트에 값 표시 (만원 단위)
            for i, row in df.iterrows():
                ax.text(row['Date'], row['Daily Sales (KRW in 10k)'], 
                        f'{row["Daily Sales (KRW in 10k)"]:.1f} (10k)', 
                        color='black', ha='center', va='bottom')  # va는 value 위치 설정 (위쪽, 아래쪽)

            # 그래프 레이블 및 제목 설정
            plt.xlabel('Date')
            plt.ylabel('Daily Sales (KRW in 10k)')
            plt.title(f'Daily Sales for the Last {days} Days ({start_date_str} to {datetime.now().strftime("%Y-%m-%d")})')
            plt.xticks(rotation=45)
            plt.grid(True)
            plt.tight_layout()
            plt.show()
        else:
            # 데이터가 없으면 로그 메시지 출력
            print(f"No sales data available for the last {days} days.")

        
    # 통계: 일주일간 요일 별 매출(막대 그래프), 수정 필요
    # def generate_weekday_sales_graph(self):
        # 오늘 날짜와 6일 전 날짜 계산
        today = datetime.today()
        one_week_ago = today - timedelta(days=6)

        # 날짜별 매출 데이터 가져오기
        cursor.execute('''
            SELECT DATE(time_stamp) as sales_date,
                SUM(CAST(REPLACE(total_price, ',', '') AS INTEGER)) as total_sales
            FROM orders
            WHERE time_stamp BETWEEN ? AND ?
            GROUP BY sales_date
            ORDER BY sales_date;
        ''', (one_week_ago.strftime('%Y-%m-%d'), today.strftime('%Y-%m-%d')))

        sales_data = cursor.fetchall()

        if sales_data:
            # DataFrame으로 변환
            df = pd.DataFrame(sales_data, columns=['Sales Date', 'Total Sales'])
            df['Sales Date'] = pd.to_datetime(df['Sales Date'])  # 날짜 형식 변환
            df['Day of Week'] = df['Sales Date'].dt.day_name()  # 요일 추출

            # 요일 순서 지정
            days_of_week = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]

            # 매출이 있는 요일만 선택
            filtered_df = df[df['Total Sales'] > 0]

            # Seaborn 막대 그래프 생성
            plt.figure(figsize=(10, 6))
            ax = sns.barplot(
                x='Day of Week', 
                y='Total Sales', 
                data=filtered_df, 
                order=[day for day in days_of_week if day in filtered_df['Day of Week'].values], 
                palette='Blues'
            )

            # 각 데이터 포인트에 값 표시
            for i, row in filtered_df.iterrows():
                ax.text(i, row['Total Sales'] + 500,  # 약간의 오프셋
                        f'{row["Total Sales"]:,}', 
                        color='black', ha='center', va='bottom')
            conn.close()
            # 그래프 제목 및 레이블
            plt.xlabel('Day of Week')
            plt.ylabel('Total Sales (KRW)')
            plt.title(f'Weekly Sales by Day ({one_week_ago.strftime("%Y-%m-%d")} to {today.strftime("%Y-%m-%d")})')
            plt.xticks(rotation=45)  # X축 레이블 회전
            plt.grid(True)
            plt.tight_layout()
            plt.show()
        else:
            print("No sales data available for the last week.")
    
    # 통계: 지난 일주일간 요일별 매출 매출 0이면 안뜸
    def generate_weekday_sales_graph(self):
        db_path = self.get_resource_file_path()
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        # 오늘 날짜와 6일 전 날짜 계산 (오늘 포함)
        today = datetime.today()
        end_of_today = (today + timedelta(days=1)).strftime('%Y-%m-%d 00:00:00')  # 내일 0시까지 포함
        one_week_ago = today - timedelta(days=6)

        # 날짜별 매출 데이터 가져오기
        cursor.execute('''
            SELECT DATE(time_stamp) as sales_date,
                SUM(CAST(REPLACE(total_price, ',', '') AS INTEGER)) as total_sales
            FROM orders
            WHERE time_stamp BETWEEN ? AND ?
            GROUP BY sales_date
            ORDER BY sales_date;
        ''', (one_week_ago.strftime('%Y-%m-%d 00:00:00'), end_of_today))

        sales_data = cursor.fetchall()

        if sales_data:
            # DataFrame으로 변환
            df = pd.DataFrame(sales_data, columns=['Sales Date', 'Total Sales'])
            df['Sales Date'] = pd.to_datetime(df['Sales Date'])  # 날짜 형식 변환
            df['Day of Week'] = df['Sales Date'].dt.day_name()  # 요일 추출

            # 오늘을 포함한 7일 데이터를 모두 포함하도록 요일 순서 지정
            days_of_week = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]

            # 매출이 있는 요일만 선택
            filtered_df = df[df['Total Sales'] > 0]

            # Seaborn 막대 그래프 생성
            plt.figure(figsize=(10, 6))
            ax = sns.barplot(
                x='Day of Week', 
                y='Total Sales', 
                data=filtered_df, 
                order=[day for day in days_of_week if day in filtered_df['Day of Week'].values], 
                palette='Blues'
            )

            # 각 데이터 포인트에 값 표시
            for i, row in filtered_df.iterrows():
                ax.text(i, row['Total Sales'] + 500,  # 약간의 오프셋
                        f'{row["Total Sales"]:,}', 
                        color='black', ha='center', va='bottom')
            conn.close()
            # 그래프 제목 및 레이블
            plt.xlabel('Day of Week')
            plt.ylabel('Total Sales (KRW)')
            plt.title(f'Weekly Sales by Day ({one_week_ago.strftime("%Y-%m-%d")} to {today.strftime("%Y-%m-%d")})')
            plt.xticks(rotation=45)  # X축 레이블 회전
            plt.grid(True)
            plt.tight_layout()
            plt.show()
        else:
            print("No sales data available for the last week.")

    # 통계: 메뉴별 매출 시각화 (날짜 범위 입력)
    def generate_menu_sales_graph(self,date):
        db_path = self.get_resource_file_path()
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        start_date, end_date = date  # date_range: 튜플 또는 리스트로 시작일, 종료일을 입력받음
    
        # 날짜 범위에 따른 메뉴별 매출 합산
        cursor.execute('''
            SELECT menu_id, 
                SUM(CAST(REPLACE(total_price, ',', '') AS INTEGER)) AS total_sales
            FROM orders
            WHERE time_stamp BETWEEN ? AND ?
            GROUP BY menu_id
            ORDER BY total_sales DESC;
        ''', (start_date, end_date))

        sales_data = cursor.fetchall()

        # 데이터가 있을 경우 메뉴별 매출 계산 및 그래프 생성
        if sales_data:
            # 메뉴 ID와 메뉴 이름 매칭하기 위한 dict 준비
            menu_names = {}
            cursor.execute('SELECT menu_id, menu_name FROM menu;')
            menu_data = cursor.fetchall()

            # menu_id와 menu_name을 매핑
            for menu in menu_data:
                menu_names[menu[0]] = menu[1]

            # pandas DataFrame으로 변환
            df = pd.DataFrame(sales_data, columns=['Menu ID', 'Total Sales'])

            # 메뉴 이름을 'Menu Name' 열에 추가
            df['Menu Name'] = df['Menu ID'].map(menu_names)

            # 메뉴 ID를 문자열로 처리하고 매출을 만 원 단위로 변환
            df['Menu ID'] = df['Menu ID'].astype(str)
            df['Total Sales (KRW in 10k)'] = df['Total Sales'] / 10000  # 만 원 단위로 변환

            # seaborn을 이용한 막대 그래프
            plt.figure(figsize=(10, 6))
            ax = sns.barplot(x='Menu Name', y='Total Sales (KRW in 10k)', data=df, palette='Blues')

            # 각 데이터 포인트에 값 표시
            for i, row in df.iterrows():
                ax.text(i, row['Total Sales (KRW in 10k)'] + 0.1,  # 약간의 오프셋을 주어 값이 막대 위에 표시되도록 함
                        f'{row["Total Sales (KRW in 10k)"]:.1f} (10k)', 
                        color='black', ha='center', va='bottom')
            conn.close()
            # 그래프 제목 및 레이블
            plt.xlabel('Menu Name')
            plt.ylabel('Total Sales (KRW in 10k)')
            plt.title(f'Menu Sales from {start_date} to {end_date}')
            plt.xticks(rotation=45)
            plt.grid(True)

            # 레이아웃 자동 조정 (그래프와 레이블 간격)
            plt.tight_layout()
            plt.show()

        else:
            # 데이터가 없으면 로그 메시지 출력
            print(f"No sales data available for the period from {start_date} to {end_date}.")
    
    # database 닫기
    # def close_database(self):
    #     conn.close()
    #     print("- Database(order_datas.db) connection closed.\n")

def main():
    # OrderManager 객체 생성 및 데이터 삽입
    node = OrderManager()

    # 요청 데이터 예시
    place_order = {
        "table_id": 5,
        "orders": [
            {"menu_id": "1", "quantity": 2},
            {"menu_id": "2", "quantity": 2},
            {"menu_id": "3", "quantity": 2},
            {"menu_id": "3", "quantity": 2}

        ]
    }
    input_menu = {
        'Sirloin cutlet': 10000,
        'Tenderloin cutlet': 11000,
        'Assorted Katsu': 16000,
        'Cheese Katsu': 13000
    }
    logs = """
    Nov 27 10:15:32 node01 kernel: [123456.789012] CPU load high: 95%
    Nov 27 10:16:01 node02 sshd[12345]: Accepted password for user1 from 192.168.0.101 port 22 ssh2
    Nov 27 10:17:10 node03 systemd[1]: Started MyService.
    Nov 27 10:18:00 node01 kernel: [123457.123456] Out of memory: Kill process 5678 (myapp) score 123 or sacrifice child
    """
    
    # 시작일과 종료일 입력받기 (YYYY-MM-DD 형식)
    date = ["2024-11-27", "2024-11-29"]

    # 데이터 삽입 실행
    node.update_menu(input_menu)
    node.insert_order(place_order)
    node.insert_log(logs)

    # 통계
    node.generate_sales_graph()
    node.generate_weekday_sales_graph()
    node.generate_menu_sales_graph(date)

    node. close_database()
    
    

if __name__ == '__main__':
    main()
