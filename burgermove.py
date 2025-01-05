#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from detect import WebcamSubscriber


class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.command = Twist()
        self.state = "STOP"
        
        self.webcam_subscriber = WebcamSubscriber()

    def timer_callback(self):
        """타이머 콜백 함수: 현재 명령을 발행"""
        self.publisher.publish(self.command)
        self.get_logger().info(f'현재 속도 명령: {self.command}')

    def move_forward(self, linear_speed=0.2, duration=2.0):
        """
        전진 이동
        :param linear_speed: 선속도 (양수: 전진, 음수: 후진)
        :param duration: 이동 시간(초)
        """
        self.command.linear.x = linear_speed
        self.command.angular.z = 0.0
        self._publish_for_duration(duration)

    def rotate(self, angular_speed=0.5, duration=2.0):
        """
        회전 이동
        :param angular_speed: 각속도 (양수: 좌회전, 음수: 우회전)
        :param duration: 회전 시간(초)
        """
        self.command.linear.x = 0.0
        self.command.angular.z = angular_speed
        self._publish_for_duration(duration)

    def stop(self, duration=1.0):
        """정지"""
        self.command.linear.x = 0.0
        self.command.angular.z = 0.0
        self._publish_for_duration(duration)

    def _publish_for_duration(self, duration):
        """
        지정된 시간 동안 명령 발행
        :param duration: 발행 시간(초)
        """
        end_time = time.time() + duration
        while rclpy.ok() and time.time() < end_time:
            self.publisher.publish(self.command)
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotController()
    
    try:
        # TurtleBot 제어 예제
        controller.get_logger().info('전진 이동 시작...')
        controller.move_forward(linear_speed=0.2, duration=3.0)
        
        controller.get_logger().info('물체 감지 시작...')
        controller.webcam_subscriber.detect(5)
        
        controller.get_logger().info('회전 시작...')
        controller.rotate(angular_speed=0.5, duration=2.0)
        
        controller.get_logger().info('정지...')
        controller.stop(duration=1.0)
    
    except KeyboardInterrupt:
        controller.get_logger().info('사용자에 의한 중단, 정지합니다...')
        controller.stop()
    except Exception as e:
        controller.get_logger().error(f'오류 발생: {str(e)}')
    finally:
        controller.webcam_subscriber.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
