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
```

#### When module test
###### Example
```bash
cd src
python3 -m ssts.GUI

```
모듈화 설계가 되어 있기 때문에, 각 모듈을 개별 테스트 할때 하위 모듈을 잘 불러오게 하려면 위와 같은 형식으로 실행시켜야 한다.

#### turn on gazebo
'''bash
cd turtlebot3_ws/
colcon build --symlink-install
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
'''

#### turn on rviz2
'''bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/your/path/serving-bot/src/map.yaml
'''
