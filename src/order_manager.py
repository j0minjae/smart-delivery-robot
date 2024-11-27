import sqlite3
from datetime import datetime

class OrderManager:
    # 일일 매출: 테이블 생성
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

    # 일일 매출: 주문수락 데이터 추출 및 삽입
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
        print("Order saved to database.")

    def close_connection(self):
        self.conn.close()
        print("Database connection closed.")


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

    # 현재 시간 생성
    time_stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    # 데이터 삽입 실행
    node.get_and_insert_order(request, menu_price, time_stamp)

    # 연결 종료
    node.close_connection()


if __name__ == '__main__':
    main()
