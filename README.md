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

4. TurtleBot3 제어 및 객체 인식 통합 시스템

#### TurtleBot3 제어 시스템 (`burgermove.py`)

##### 시스템 구성
```python
from detect import WebcamSubscriber

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.webcam_subscriber = WebcamSubscriber()  # YOLO 객체 감지 통합
```

##### 자동 주행 시퀀스
```python
def main(args=None):
    try:
        # 1. 전진 이동
        controller.move_forward(linear_speed=0.2, duration=3.0)
        
        # 2. 물체 감지 수행
        controller.webcam_subscriber.detect(5)  # 5초간 감지
        
        # 3. 회전 이동
        controller.rotate(angular_speed=0.5, duration=2.0)
        
        # 4. 안전 정지
        controller.stop(duration=1.0)
    except KeyboardInterrupt:
        controller.stop()  # 안전한 종료
```

##### 주요 기능
1. 이동 제어
   - 전진/후진: `move_forward(linear_speed, duration)`
   - 회전: `rotate(angular_speed, duration)`
   - 정지: `stop(duration)`

2. 객체 감지 통합
   - WebcamSubscriber 클래스와 통합
   - 정지 상태에서 자동 감지 수행
   - 감지 완료 후 다음 동작 진행

3. 안전 기능
   - 타이머 기반 명령 실행
   - 예외 상황 처리
   - 안전한 종료 절차

##### 동작 시퀀스
1. 초기화 단계
   - ROS2 노드 초기화
   - 속도 제어 퍼블리셔 생성
   - YOLO 객체 감지 시스템 초기화

2. 주행 단계
   - 3초간 전진 이동 (0.2m/s)
   - 정지 후 5초간 물체 감지
   - 2초간 회전 (0.5rad/s)
   - 1초간 안전 정지

3. 종료 단계
   - 모든 동작 완료 후 정지
   - 노드 정리 및 종료
   - 시스템 자원 해제

##### 에러 처리
- 키보드 인터럽트 처리
- 예외 상황 로깅
- 안전한 종료 보장

#### ArUco 마커 감지 (`aruco_sub.py`)
- 4x4 ArUco 마커 실시간 감지
- 마커 정보 처리:
  - ID 인식
  - 위치 좌표 계산
  - 바운딩 박스 표시
- 자동 이미지 저장:
  - 위치: `aruco_images` 디렉토리
  - 파일명: `marker_[ID]_[날짜시간].jpg`
  - 저장 간격: 1초

#### 통합 기능
- 실시간 영상 처리
- 동시 다중 객체 감지
- 자동 데이터 수집
- 안전한 예외 처리:
  - 키보드 인터럽트 처리
  - 에러 로깅
  - 리소스 정리

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

### YOLO 객체 감지 시스템 (`detect.py`)

#### 객체 감지 및 이미지 저장 기능
- WebcamSubscriber 클래스를 통한 통합 구현:
  ```python
  from detect import WebcamSubscriber
  
  # TurtleBot 제어 클래스에서 사용
  self.webcam_subscriber = WebcamSubscriber()
  self.webcam_subscriber.detect(5)  # 5초간 객체 감지 수행
  ```

#### 주요 기능
1. 실시간 객체 감지
   - YOLOv8n 모델 사용
   - 압축 이미지 토픽 구독 (`/webcam/image/compressed`)
   - 실시간 객체 인식 및 분류

2. 자동 이미지 저장
   - 저장 위치: `detect` 디렉토리
   - 파일명: `[객체명].jpg`
   - 중복 객체 제외 (set 자료구조 활용)
   - 감지된 객체별 한 번만 저장

3. 시각화 기능
   - 실시간 바운딩 박스 표시
   - 객체 클래스명 표시
   - 객체 감지 결과 실시간 화면 출력

4. 제어 기능
   - 지정된 시간 동안 감지 수행
   - 감지 완료 자동 알림
   - 안전한 종료 처리

#### 작동 순서
1. 객체 감지 초기화
   - YOLO 모델 로드
   - 저장 디렉토리 생성
   - 윈도우 생성

2. 감지 모드 활성화
   - 시간 설정 (기본 5초)
   - 감지 상태 초기화
   - 중복 객체 필터링 초기화

3. 이미지 처리
   - 압축 이미지 디코딩
   - YOLO 모델로 객체 감지
   - 감지된 객체 이미지 추출

4. 결과 저장
   - 새로운 객체만 선별
   - 이미지 파일로 저장
   - 로그 메시지 출력

#### 통합 시스템 연동
- TurtleBot3 제어와 연계
- 정지 시점에서 자동 감지 수행
- 감지 완료 후 다음 동작 진행
