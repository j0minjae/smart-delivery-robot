# serving-bot

## Quick Start
#### interface build and apply
```bash
colcon build --symlink-install --packages-select serving_bot_interfaces
source install/setup.bash
```


#### show database 
```bash
cd ~/serving-bot
pip install -r requirements.txt
python3 src/restaurant_orders.py
sqlite3 database/order_datas.db # 터미널 1
sqlite3 database/log_datas.db # 터미널 2
sqlite3 database/menu.db # 터미널3

sqlite> .headers on
sqlite> .mode column
sqlite> SELECT * FROM orders; # 터미널 1
sqlite> SELECT * FROM logs; # 터미널 2
sqlite3 SELECT * FROM menu; # 터미널3
sqlite> .quit # sqlite 종료

