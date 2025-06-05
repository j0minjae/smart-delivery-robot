# serving-bot

## 프로젝트 개요
스마트 딜리버리 로봇은 ROS2를 기반으로 자율주행과 음식 배달 기능을 수행하는 서비스 로봇 개발 프로젝트입니다.
식당 환경을 간단하게 묘사한 시뮬레이션에서 TutleBot로봇이 정화하게 경로를 따라가고 장애물을 회피합니다.

## 주요 역할 및 기여
- ROS2 s

## 사용 기술
- Python, ROS2 Humble  
- OpenCV, YOLOv8,SQLite 
- Git, GitHub

## 사용 기술
- Python, ROS2 Humble  
- OpenCV, YOLOv8, Deep SORT  
- Git, GitHub
  
## 결과 및 성과
- 장애물 인식 정확도 90% 이상 달성  
- 시뮬레이션 환경에서 안정적 경로 주행 성공  
- 실시간 원격 모니터링 시스템 구축

## 향후 계획
- 멀티로봇 협업 기능 추가

## 참고 자료
- [ROS2 공식 문서](https://docs.ros.org/en/rolling/)
- [Gazebo 공식 문서](
  
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
