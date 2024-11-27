class DB:
    def __init__(self):
        self.db = {}
    
    def update_db(self, table_id, order):
        self.db[table_id] = order
        return print(self.db)
    
def main():
    db = DB()

if __name__ == '__main__':
    main()