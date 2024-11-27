import sqlite3
from datetime import datetime

class LogManager:
    # 로그 데이터: 테이블 생성
    def __init__(self):
        self.conn = sqlite3.connect('database/log_datas.db')  # SQLite DB 연결
        self.cursor = self.conn.cursor()
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS logs (
                log_id INTEGER PRIMARY KEY AUTOINCREMENT,
                time_stamp TEXT,
                node_name TEXT,
                log_level TEXT,
                message TEXT
            );
        ''')
        self.conn.commit()
        print("Logs table created (if it did not exist).")

    # 로그 데이터 삽입
    def insert_log(self, node_name, log_level, message):
        time_stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.cursor.execute('''
            INSERT INTO logs (time_stamp, node_name, log_level, message)
            VALUES (?, ?, ?, ?);
        ''', (time_stamp, node_name, log_level, message))
        self.conn.commit()
        print(f"Log saved: {time_stamp} - {node_name} - {log_level} - {message}")

    def close_connection(self):
        self.conn.close()
        print("Database connection closed.")


def main():
    # LogManager 객체 생성 및 로그 삽입
    log_manager = LogManager()

    # 로그 데이터 삽입 예시
    log_manager.insert_log("Node1", "INFO", "System initialized successfully.")
    log_manager.insert_log("Node2", "WARNING", "Disk space is running low.")
    log_manager.insert_log("Node3", "ERROR", "Failed to connect to the database.")

    # 연결 종료
    log_manager.close_connection()


if __name__ == '__main__':
    main()
