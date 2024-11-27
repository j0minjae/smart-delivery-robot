class log():
    def __init__(self):
        self.log_lst = []

    def update_log(self, log):
        self.log_lst.append(log)
        print(log)
        return print(self.log_lst)
    
def main():
    l = log()

if __name__ == '__main__':
    main()