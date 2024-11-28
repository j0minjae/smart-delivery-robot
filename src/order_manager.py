import sqlite3
from datetime import datetime, timedelta
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

class OrderManager:
    # 테이블: 생성
    def __init__(self):
        pass

    # 테이블: 메뉴 구성
    def update_menu(self, input_menu):
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
        # SQLite DB 연결
        conn = sqlite3.connect('database/database.db')
        cursor = conn.cursor()

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
        print("- Database(menu) updated with input menu.")

        # 연결 종료
        conn.close()
        print("- Database(menu) connection closed.\n")

    # 테이블: 주문수락 데이터 추출 및 삽입
    def insert_order(self, place_order):
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
        conn = sqlite3.connect('database/database.db')  # 주문 데이터베이스 연결
        cursor = conn.cursor()

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

        if menu_count == 0:
            print("Menu is empty. No orders can be processed.")
            conn.close()
            print("Database(menu) connection closed.")
            return

        # 주문 저장 (테이블 아이디, 메뉴 이름, 수량)
        table_id = place_order.table_id
        orders = place_order.orders

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
        print("- Order saved to database.")
        conn.close()
        print("- Database(order_datas.db) connection closed.\n")


# 통계: 지난 한 달간의 일일매출을 시각화(꺾은선 그래프) n일간의 일일매출로 바꿀수 있음
def generate_sales_graph():
    conn = sqlite3.connect('database/database.db')  # SQLite DB 연결
    cursor = conn.cursor()
    
    # 오늘 날짜 기준으로 한 달 전 날짜 계산
    one_month_ago = datetime.now() - timedelta(days=30)
    one_month_ago_str = one_month_ago.strftime('%Y-%m-%d')

    # 한 달 간의 매출 합산: 날짜별로 총 매출 계산
    cursor.execute('''
        SELECT strftime('%Y-%m-%d', time_stamp) AS date, 
               SUM(CAST(REPLACE(total_price, ',', '') AS INTEGER)) AS daily_sales
        FROM orders
        WHERE time_stamp >= ?
        GROUP BY date
        ORDER BY date;
    ''', (one_month_ago_str,))

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

    #     # 지난 한 달간의 기간 표시
    #     plt.xlabel('Date')
    #     plt.ylabel('Daily Sales (KRW in 10k)')
    #     plt.title(f'Daily Sales for the Last Month ({one_month_ago_str} to {datetime.now().strftime("%Y-%m-%d")})')
    #     plt.xticks(rotation=45)
    #     plt.grid(True)
    #     plt.tight_layout()
    #     plt.show()
    # else:
    #     # 데이터가 없으면 로그 메시지 출력
    #     print("No sales data available for the last month.")
    
    conn.close()


# 통계: 일주일간 요일 별 매출(막대 그래프), 수정 필요
def generate_weekday_sales_graph():
    # 현재 날짜
    today = datetime.today()
    tomorrow = today + timedelta(days=1)
    
    # 일주일 전 날짜 계산
    one_week_ago = today - timedelta(days=6)
    
    # SQLite DB 연결
    conn = sqlite3.connect('database/database.db')
    cursor = conn.cursor()

    # 현재 날짜부터 일주일간의 주문 데이터 조회
    cursor.execute('''
        SELECT time_stamp, total_price
        FROM orders
        WHERE time_stamp BETWEEN ? AND ?
    ''', (one_week_ago.strftime('%Y-%m-%d'), tomorrow.strftime('%Y-%m-%d')))
    
    order_data = cursor.fetchall()

    # 요일 순서 (월요일부터 일요일까지)
    days_of_week = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]
    
    # 각 요일별 매출을 초기화 (0으로 설정)
    weekly_sales = {day: 0 for day in days_of_week}

    # 데이터가 있을 경우 매출 계산
    print(order_data)
    if order_data:
        # DataFrame으로 변환
        df = pd.DataFrame(order_data, columns=['Time Stamp', 'Total Price'])
        
        # time_stamp를 datetime으로 변환
        df['Time Stamp'] = pd.to_datetime(df['Time Stamp'])
        
        # 요일별로 매출 합산
        df['Day of Week'] = df['Time Stamp'].dt.day_name()  # 요일 이름 추출
        df['Total Price'] = df['Total Price'].apply(lambda x: int(x.replace(',', '')))  # 가격의 ',' 제거하고 정수로 변환
        
        # 요일별 매출 합산
        sales_by_day = df.groupby('Day of Week')['Total Price'].sum().reset_index()

        # 매출 데이터를 weekly_sales 딕셔너리에 업데이트
        for i, row in sales_by_day.iterrows():
            weekly_sales[row['Day of Week']] = row['Total Price']

    # 오늘의 매출이 없으면 0으로 설정 (오늘 매출을 포함)
    if 'Today' not in weekly_sales:
        weekly_sales['Today'] = 0

    # DataFrame으로 변환 (요일과 매출)
    sales_by_day = pd.DataFrame(list(weekly_sales.items()), columns=['Day of Week', 'Total Sales'])

    # 요일 순서대로 정렬 (월요일부터 일요일까지)
    sales_by_day['Day of Week'] = pd.Categorical(sales_by_day['Day of Week'], categories=days_of_week + ['Today'], ordered=True)
    sales_by_day = sales_by_day.sort_values('Day of Week')

    # seaborn을 이용한 막대 그래프
    plt.figure(figsize=(10, 6))
    ax = sns.barplot(x='Day of Week', y='Total Sales', data=sales_by_day, palette='Blues')

    # 각 데이터 포인트에 값 표시
    for i, row in sales_by_day.iterrows():
        ax.text(i, row['Total Sales'] + 1000,  # 약간의 오프셋을 주어 값이 막대 위에 표시되도록 함
                f'{row["Total Sales"]:,}', 
                color='black', ha='center', va='bottom')

    # 그래프 제목 및 레이블
    plt.xlabel('Day of Week')
    plt.ylabel('Total Sales (KRW)')
    plt.title(f'Weekly Sales from {one_week_ago.strftime("%Y-%m-%d")} to {today.strftime("%Y-%m-%d")}')
    plt.xticks(rotation=45)
    
    # y축 최소값을 0으로 설정
    plt.ylim(0, sales_by_day['Total Sales'].max() * 1.1)  # 최대값은 데이터의 최대값보다 약간 크게 설정

    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # 연결 종료
    conn.close()





# 메뉴별 매출 시각화 (날짜 범위 입력)
def generate_menu_sales_graph(start_date, end_date):
    # 주문내역 데이터베이스(order_datas) 연결
    conn = sqlite3.connect('database/database.db')  # SQLite DB 연결
    cursor = conn.cursor()

    ### 날짜 범위에 따른 메뉴별 매출 합산
    cursor.execute('''
        SELECT menu_id, 
               SUM(CAST(REPLACE(total_price, ',', '') AS INTEGER)) AS total_sales
        FROM orders
        WHERE time_stamp BETWEEN ? AND ?
        GROUP BY menu_id
        ORDER BY total_sales DESC;
    ''', (start_date, end_date))

    sales_data = cursor.fetchall()

    ### 데이터가 있을 경우 메뉴별 매출 계산 및 그래프 생성
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

        # # seaborn을 이용한 막대 그래프
        # plt.figure(figsize=(10, 6))
        # ax = sns.barplot(x='Menu Name', y='Total Sales (KRW in 10k)', data=df, palette='Blues')

        # # 각 데이터 포인트에 값 표시
        # for i, row in df.iterrows():
        #     ax.text(i, row['Total Sales (KRW in 10k)'] + 0.1,  # 약간의 오프셋을 주어 값이 막대 위에 표시되도록 함
        #             f'{row["Total Sales (KRW in 10k)"]:.1f} (10k)', 
        #             color='black', ha='center', va='bottom')

        # # 그래프 제목 및 레이블
        # plt.xlabel('Menu Name')
        # plt.ylabel('Total Sales (KRW in 10k)')
        # plt.title(f'Menu Sales from {start_date} to {end_date}')
        # plt.xticks(rotation=45)
        # plt.grid(True)
        # plt.tight_layout()
        # plt.show()

    else:
        # 데이터가 없으면 로그 메시지 출력
        print(f"No sales data available for the period from {start_date} to {end_date}.")
    
    conn.close()

# def generate_menu_sales_graph(start_date, end_date):
#     # 주문내역 데이터베이스(order_datas) 연결
#     conn = sqlite3.connect('database/database.db')  # SQLite DB 연결
#     cursor = conn.cursor()

#     ### 날짜 범위에 따른 메뉴별 매출 합산
#     cursor.execute('''
#         SELECT menu_id, 
#                SUM(CAST(REPLACE(total_price, ',', '') AS INTEGER)) AS total_sales
#         FROM orders
#         WHERE time_stamp BETWEEN ? AND ?
#         GROUP BY menu_id
#         ORDER BY total_sales DESC;
#     ''', (start_date, end_date))

#     sales_data = cursor.fetchall()

#     ### 메뉴 ID에 해당하는 메뉴 이름을 가져오기
#     menu_names = {}
#     conn = sqlite3.connect('database/menu.db')  # SQLite DB 연결
#     cursor = conn.cursor()
#     cursor.execute('SELECT menu_id, menu_name FROM menu')
#     for menu_id, menu_name in cursor.fetchall():
#         menu_names[menu_id] = menu_name

#     ### 데이터가 있을 경우 메뉴별 매출 계산 및 그래프 생성
#     if sales_data:
#         # pandas DataFrame으로 변환
#         df = pd.DataFrame(sales_data, columns=['Menu ID', 'Total Sales'])
        
#         # 메뉴 ID를 문자열로 처리하고 매출을 만 원 단위로 변환
#         df['Menu Name'] = df['Menu ID'].map(menu_names)  # menu_id에 해당하는 menu_name 매핑
#         df['Total Sales (KRW in 10k)'] = df['Total Sales'] / 10000  # 만 원 단위로 변환

#         # 메뉴 이름을 x축에 표시할 수 있도록 정렬
#         df = df.sort_values(by='Total Sales', ascending=False)  # 매출 합계 기준 내림차순 정렬

#         # seaborn을 이용한 막대 그래프
#         plt.figure(figsize=(10, 6))
#         ax = sns.barplot(x='Menu Name', y='Total Sales (KRW in 10k)', data=df, palette='Blues')
        
#         # 각 데이터 포인트에 값 표시
#         for i, row in df.iterrows():
#             ax.text(i, row['Total Sales (KRW in 10k)'], 
#                     f'{row["Total Sales (KRW in 10k)"]:.1f} (10k)', 
#                     color='black', ha='center', va='bottom')

#         # 그래프 제목 및 레이블
#         plt.xlabel('Menu Name')
#         plt.ylabel('Total Sales (KRW in 10k)')
#         plt.title(f'Menu Sales from {start_date} to {end_date}')
#         plt.xticks(rotation=45)
#         plt.grid(True)
#         plt.tight_layout()
#         plt.show()

#     else:
#         # 데이터가 없으면 로그 메시지 출력
#         print(f"No sales data available for the period from {start_date} to {end_date}.")
    
#     conn.close()

def main():
    # OrderManager 객체 생성 및 데이터 삽입
    node = OrderManager()

    # 요청 예시 데이터
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
    
    # 시작일과 종료일 입력받기 (YYYY-MM-DD 형식)
    start_date = "2024-11-27"
    end_date = "2024-11-29" # 28일을 포함하지 않음

    # 데이터 삽입 실행
    # node.update_menu(input_menu)
    # node.insert_order(place_order)

    # 통계
    # generate_sales_graph()
    generate_weekday_sales_graph()
    # generate_menu_sales_graph(start_date, end_date)

if __name__ == '__main__':
    main()
