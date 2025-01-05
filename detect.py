#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO
import os
import time

class WebcamSubscriber(Node):
    def __init__(self):
        super().__init__('webcam_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'webcam/image/compressed',
            self.image_callback,
            10)
        
        # YOLO 모델 로드
        self.model = YOLO('yolov8n.pt')
        
        # 감지 관련 변수
        self.detect_mode = False
        self.detect_start_time = 0
        self.detect_duration = 0
        self.detected_objects = set()  # 감지된 객체를 set으로 변경하여 중복 방지
        
        # 저장 경로 생성
        self.save_dir = 'detect'
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        # 창 생성
        cv2.namedWindow('Webcam View', cv2.WINDOW_NORMAL)
        
        self.detection_complete = False
        
    def detect(self, duration):
        """지정된 시간 동안 물체 감지 모드 활성화"""
        self.detect_mode = True
        self.detect_start_time = time.time()
        self.detect_duration = duration
        self.detected_objects = set()
        self.detection_complete = False
        self.get_logger().info(f'{duration}초 동안 물체 감지를 시작합니다.')
        
        # 감지가 완료될 때까지 대기
        while not self.detection_complete and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
    def image_callback(self, msg):
        # 압축된 이미지 데이터를 numpy 배열로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if self.detect_mode:
            # 감지 시간 확인
            if time.time() - self.detect_start_time > self.detect_duration:
                self.detect_mode = False
                self.detection_complete = True
                self.get_logger().info('물체 감지가 완료되었습니다.')
                return
            
            # YOLO로 물체 감지
            results = self.model(image)
            
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    # 클래스 이름 가져오기
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names[cls_id]
                    
                    # 이미 감지된 물체는 건너뛰기
                    if cls_name in self.detected_objects:
                        continue
                    
                    # 박스 좌표 가져오기
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    
                    # 물체 이미지 추출 및 저장
                    obj_img = image[y1:y2, x1:x2]
                    if obj_img.size > 0:  # 이미지가 유효한 경우
                        # 파일 이름 생성 및 저장
                        filename = f"{cls_name}.jpg"
                        save_path = os.path.join(self.save_dir, filename)
                        cv2.imwrite(save_path, obj_img)
                        self.detected_objects.add(cls_name)  # 감지된 물체 목록에 추가
                        self.get_logger().info(f'물체 저장됨: {filename}')
                    
                    # 화면에 박스 그리기
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(image, cls_name, (x1, y1 - 10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        # 이미지 표시
        cv2.imshow('Webcam View', image)
        cv2.waitKey(1)
    
    def __del__(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    webcam_subscriber = WebcamSubscriber()
    
    try:
        rclpy.spin(webcam_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        webcam_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()