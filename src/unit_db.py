class DB:
    def __init__(self):
        self.db = {}
    
    def update_db(self, table_id, orders):
        self.db[table_id] = orders
        return print(self.db)
    
def main():
    db = DB()

if __name__ == '__main__':
    main()