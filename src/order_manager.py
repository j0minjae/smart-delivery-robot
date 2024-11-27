import sqlite3
from datetime import datetime, timedelta
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

class OrderManager:
    # 테이블: 생성
    def __init__(self):
        self.conn = sqlite3.connect('database/order_chit.db')  # SQLite DB 연결
        self.cursor = self.conn.cursor()
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS orders (
                order_id INTEGER PRIMARY KEY AUTOINCREMENT,
                time_stamp TEXT,
                table_id INTEGER,
                menu_id TEXT,
                total_price TEXT
            );
        ''')
        self.conn.commit()
        print("Table created (if it did not exist).")

    # 테이블: 주문수락 데이터 추출 및 삽입
    def get_and_insert_order(self, place_order, menu_price, time_stamp):
        # 메뉴 개수 만큼 order 저장 (테이블 아이디, 메뉴이름, 수량)
        table_id = place_order['table_id']
        orders = place_order['orders']
        for order in orders:
            menu_id = str(order['menu_id'])
            quantity = order['quantity']
            price = menu_price[menu_id] * quantity
            formatted_price = '{:,}'.format(price)

            self.cursor.execute('''
                INSERT INTO orders (time_stamp, table_id, menu_id, total_price)
                VALUES (?, ?, ?, ?);
            ''', (time_stamp, table_id, menu_id, formatted_price))
        
        self.conn.commit()
        # order 저장 인포 로깅
        print("Order saved to database.")
        
    def close_connection(self):
        self.conn.close()
        print("Database connection closed.")

# 통계: 지난 한 달간의 일일매출을 시각화(꺾은선 그래프)
def generate_sales_graph():
    conn = sqlite3.connect('database/order_chit.db')  # SQLite DB 연결
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

        # 지난 한 달간의 기간 표시
        plt.xlabel('Date')
        plt.ylabel('Daily Sales (KRW in 10k)')
        plt.title(f'Daily Sales for the Last Month ({one_month_ago_str} to {datetime.now().strftime("%Y-%m-%d")})')
        plt.xticks(rotation=45)
        plt.grid(True)
        plt.tight_layout()
        plt.show()
    else:
        # 데이터가 없으면 로그 메시지 출력
        print("No sales data available for the last month.")
    
    conn.close()


# 통계: 일주일간 요일 별 매출(막대 그래프), 수정 필요
# def generate_weekday_sales_graph():
#     conn = sqlite3.connect('database/order_chit.db')  # SQLite DB 연결
#     cursor = conn.cursor()
    
#     # 오늘 날짜 기준으로 7일 전 날짜 계산
#     one_week_ago = datetime.now() - timedelta(days=7)
#     one_week_ago_str = one_week_ago.strftime('%Y-%m-%d')

#     # 일주일 간의 매출 합산: 요일별로 총 매출 계산
#     cursor.execute('''
#         SELECT strftime('%Y-%m-%d', time_stamp) AS date, 
#                strftime('%w', time_stamp) AS weekday,  -- 요일을 숫자로 추출 (0: 일요일, 1: 월요일, ...)
#                total_price
#         FROM orders
#         WHERE time_stamp >= ?
#         ORDER BY date;
#     ''', (one_week_ago_str,))

#     sales_data = cursor.fetchall()  # 리스트로 저장 
#     print("Sales Data:", sales_data)  # sales_data 확인

#     # 데이터가 있을 경우 요일별 매출 계산 및 그래프 생성
#     if sales_data:
#         # pandas DataFrame으로 변환
#         df = pd.DataFrame(sales_data, columns=['Date', 'Weekday', 'Total Price'])

#         # 쉼표 제거 및 숫자 변환
#         df['Total Price'] = df['Total Price'].str.replace(',', '', regex=False).astype(float)
#         print("DataFrame after price conversion:", df)  # 데이터프레임 확인

#         # 만 원 단위로 변환한 'Daily Sales' 컬럼 추가
#         df['Daily Sales (KRW in 10k)'] = df['Total Price'] / 10000
#         print("DataFrame with Daily Sales:", df)  # Daily Sales 컬럼 확인

#         # 요일별 매출 합산
#         weekday_sales = df.groupby('Weekday')['Daily Sales (KRW in 10k)'].sum().reset_index()

#         # 요일 순서대로 DataFrame을 고정
#         weekdays = [0, 1, 2, 3, 4, 5, 6]  # 0: 일요일, 1: 월요일, ..., 6: 토요일
#         weekday_sales = weekday_sales.set_index('Weekday').reindex(weekdays).reset_index()

#         # NaN 값을 0으로 처리
#         weekday_sales['Daily Sales (KRW in 10k)'] = weekday_sales['Daily Sales (KRW in 10k)'].fillna(0)
#         print("Weekday Sales after filling NaN:", weekday_sales)  # NaN 처리 후 데이터 확인

#         # seaborn을 이용한 막대 그래프
#         plt.figure(figsize=(10, 6))
#         ax = sns.barplot(x='Weekday', y='Daily Sales (KRW in 10k)', data=weekday_sales, palette='Blues')

#         # 각 데이터 포인트에 값 표시 (만원 단위)
#         for i, row in weekday_sales.iterrows():
#             ax.text(i, row['Daily Sales (KRW in 10k)'], 
#                     f'{row["Daily Sales (KRW in 10k)"]:.1f} (10k)', 
#                     color='black', ha='center', va='bottom')

#         # Y축의 레이블을 만 원 단위로 표시
#         plt.xlabel('Day of the Week')
#         plt.ylabel('Total Sales (KRW in 10k)')
#         plt.title(f'Weekday Sales for the Last Week ({one_week_ago_str} to {datetime.now().strftime("%Y-%m-%d")})')

#         # X축 레이블을 회전하여 가독성 높이기
#         plt.xticks(rotation=45)

#         # X축 레이블을 숫자로 설정 (0: 일요일, 1: 월요일, ...)
#         plt.xticks(ticks=range(7), labels=['0', '1', '2', '3', '4', '5', '6'])

#         # 그리드 추가
#         plt.grid(True)

#         # 그래프 레이아웃을 최적화
#         plt.tight_layout()

#         # 그래프 출력
#         plt.show()
#     else:
#         # 데이터가 없으면 로그 메시지 출력
#         print("No sales data available for the last week.")

#     conn.close()





# 메뉴별 매출 시각화 (날짜 범위 입력)
def generate_menu_sales_graph(start_date, end_date):
    conn = sqlite3.connect('database/order_chit.db')  # SQLite DB 연결
    cursor = conn.cursor()

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
        # pandas DataFrame으로 변환
        df = pd.DataFrame(sales_data, columns=['Menu ID', 'Total Sales'])
        
        # 메뉴 ID를 문자열로 처리하고 매출을 만 원 단위로 변환
        df['Menu ID'] = df['Menu ID'].astype(str)
        df['Total Sales (KRW in 10k)'] = df['Total Sales'] / 10000  # 만 원 단위로 변환

        # seaborn을 이용한 막대 그래프
        plt.figure(figsize=(10, 6))
        ax = sns.barplot(x='Menu ID', y='Total Sales (KRW in 10k)', data=df, palette='Blues')
        
        # 각 데이터 포인트에 값 표시
        for i, row in df.iterrows():
            ax.text(row.name, row['Total Sales (KRW in 10k)'], 
                    f'{row["Total Sales (KRW in 10k)"]:.1f} (10k)', 
                    color='black', ha='center', va='bottom')

        # 그래프 제목 및 레이블
        plt.xlabel('Menu ID')
        plt.ylabel('Total Sales (KRW in 10k)')
        plt.title(f'Menu Sales from {start_date} to {end_date}')
        plt.xticks(rotation=45)
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    else:
        # 데이터가 없으면 로그 메시지 출력
        print(f"No sales data available for the period from {start_date} to {end_date}.")
    
    conn.close()

def main():
    # OrderManager 객체 생성 및 데이터 삽입
    node = OrderManager()

    # 메뉴와 가격 정보
    menu_price = {
        "1": 10000,
        "2": 11000,
        "3": 12000
    }

    # 요청 예시 데이터
    request = {
        "table_id": 5,
        "orders": [
            {"menu_id": "1", "quantity": 2},
            {"menu_id": "2", "quantity": 2},
            {"menu_id": "3", "quantity": 2}
        ]
    }

    # 시작일과 종료일 입력받기 (YYYY-MM-DD 형식)
    start_date = "2024-11-01"
    end_date = "2024-11-28" # 28일을 포함하지 않음

    # 현재 시간 생성
    time_stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    # 데이터 삽입 실행
    node.get_and_insert_order(request, menu_price, time_stamp)

    # 연결 종료
    node.close_connection()

    # 통계
    generate_sales_graph()
    # generate_weekday_sales_graph()
    generate_menu_sales_graph(start_date, end_date)

if __name__ == '__main__':
    main()
