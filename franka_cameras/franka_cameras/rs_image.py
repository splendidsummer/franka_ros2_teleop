# filepath: your_pkg/src/image_listener.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageListener(Node):
    def __init__(self):
        super().__init__('image_listener')
        # 创建 CvBridge 对象，用于在 ROS Image 与 OpenCV 图像之间转换
        self.bridge = CvBridge()
        # 订阅彩色图像话题
        self.sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.callback,
            10
        )

    def callback(self, msg):
        # 将 ROS Image 消息转换为 OpenCV BGR 图像
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # 将图像缩放到 224x224
        resized_img = cv2.resize(
            cv_img,
            (224, 224),
            interpolation=cv2.INTER_AREA
        )
        # 打印调整后图像的形状和数据类型
        self.get_logger().info(
            f"已调整图像大小，形状: {resized_img.shape}, 数据类型: {resized_img.dtype}"
        )
        # 显示缩放后的图像
        cv2.imshow('Resized Color', resized_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()