#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO

class YOLOSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'webcam/image/compressed',
            self.image_callback,
            10)
        
        # YOLO 모델 로드
        self.model = YOLO('yolov8n.pt')
        
        # 창 생성
        cv2.namedWindow('YOLO Detection', cv2.WINDOW_NORMAL)
        
    def image_callback(self, msg):
        # 압축된 이미지 데이터를 numpy 배열로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # YOLO로 객체 감지 수행
        results = self.model(image)
        
        # 결과 시각화
        annotated_frame = results[0].plot()
        
        # 이미지 표시
        cv2.imshow('YOLO Detection', annotated_frame)
        cv2.waitKey(1)
    
    def __del__(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    yolo_subscriber = YOLOSubscriber()
    
    try:
        rclpy.spin(yolo_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()