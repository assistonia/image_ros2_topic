# ROS2 웹캠 ArUco 마커 인식 시스템

이 프로젝트는 ROS2를 사용하여 웹캠으로 ArUco 마커를 인식하고 처리하는 시스템과 YOLO 객체 감지를 포함한 시스템입니다.

## 시스템 구성
- 로봇 컴퓨터: 웹캠 연결 및 영상 발행
- 제어 컴퓨터: 마커 인식 및 처리

## 필수 요구사항

1. ROS2 Humble (로봇 컴퓨터 및 제어 컴퓨터 모두)
2. ros2 humble의 파이썬 인터프리터 사용
3. 웹캠 (로봇 컴퓨터에 연결)
4. 필요한 Python 패키지 설치 (제어 컴퓨터):

```bash
pip3 install -r requirements.txt
```

## 실행 방법

### 로봇 컴퓨터에서:
1. 웹캠 영상 발행
```bash
bash webcam.sh
```

### 제어 컴퓨터에서:
1. 웹캠 영상 구독 및 표시
```bash
python3 webcam_subscriber.py
```

2. ArUco 마커 인식 실행
```bash
python3 aruco_sub.py
```

3. YOLO 객체 감지 실행
```bash
python3 yolo_sub.py
```

## 프로젝트 구조

로봇 컴퓨터:
- `webcam_topic.py`: 웹캠 영상을 ROS2 토픽으로 발행
- `webcam.sh`: 웹캠 발행 노드 실행 스크립트

제어 컴퓨터:
- `webcam_subscriber.py`: 웹캠 영상을 구독하여 화면에 표시
- `aruco_sub.py`: ArUco 마커 인식 및 처리
- `yolo_sub.py`: YOLO 객체 감지 처리
- `turtlebot.sh`: TurtleBot3 실행 스크립트

## 주요 기능 (제어 컴퓨터)

1. 웹캠 영상 실시간 스트리밍
2. ArUco 마커 실시간 인식
3. YOLO 실시간 객체 감지
4. 인식된 마커 정보 표시
   - 마커 ID
   - 바운딩 박스
   - 중심점 좌표
5. 마커 인식 시 자동 이미지 저장 (1초 간격)

## 저장된 이미지

인식된 ArUco 마커 이미지는 제어 컴퓨터의 `aruco_images` 디렉토리에 자동으로 저장됩니다.
파일명 형식: `marker_[ID]_[날짜시간].jpg`
