import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
# vision_msgs 임포트
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        # Detection(박스용), Segmentation(세그/마스크용) 모델 분리 가능
        self.model = YOLO('yolov8n-seg.pt')  # segmentation까지 지원하는 가중치
        self.model.to('cpu')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/depth_camera/image_raw',
            self.image_callback,
            10)
        
        # Detection2DArray (bbox) 발행
        self.bbox_publisher = self.create_publisher(Detection2DArray, '/detection/bounding_boxes', 10)
        self.result_image_publisher = self.create_publisher(Image, '/detection/image_result', 10)
        # Segmentation mask 이미지 발행
        self.segmentation_publisher = self.create_publisher(Image, '/detection/segmentation', 10)
        self.get_logger().info('Object Detector Node has been started.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            results = self.model(cv_image, verbose=False)
            
            self.process_detection(results, msg)
            self.process_segmentation(results, msg)

            # Annotated(박스+마스크) 이미지 송출
            annotated_frame = results[0].plot()
            result_msg = self.bridge.cv2_to_imgmsg(annotated_frame, 'rgb8')
            self.result_image_publisher.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def process_detection(self, results, msg):
        # Detection2DArray 메시지 생성 및 publish
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        for r in results:
            boxes = r.boxes
            for box in boxes:
                detection = Detection2D()
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = str(self.model.names[int(box.cls)])
                hypothesis.score = float(box.conf)
                detection.results.append(hypothesis)
                # Bounding Box 중심 및 크기
                xywh = box.xywh.cpu().numpy()[0]
                detection.bbox.center.x = float(xywh[0])
                detection.bbox.center.y = float(xywh[1])
                detection.bbox.size_x = float(xywh[2])
                detection.bbox.size_y = float(xywh[3])
                detection_array.detections.append(detection)
        self.bbox_publisher.publish(detection_array)

    def process_segmentation(self, results, msg):
        """
        YOLOv8 segmentation 결과를 활용해 마스크 이미지를 추출/발행
        """
        import numpy as np
        if hasattr(results[0], 'masks') and results[0].masks is not None:
            # 각 객체별 마스크(0-1 배열), shape: [N, H, W]
            masks = results[0].masks.data.cpu().numpy()  # (N, H, W)
            # 객체 클래스별 색 또는 마스킹 지정 (예시: 각 마스크의 픽셀을 255*N 등으로 구분)
            composite_mask = np.zeros_like(masks[0], dtype=np.uint8)
            for idx, mask in enumerate(masks):
                # (참고: idx별 색/값 지정, 여러 객체가 겹치면 마지막 idx가 우선)
                composite_mask[mask > 0.5] = 50 + idx * 30  # 임의 색
            mask_msg = self.bridge.cv2_to_imgmsg(composite_mask, 'mono8')
            mask_msg.header = msg.header
            self.segmentation_publisher.publish(mask_msg)
        else:
            self.get_logger().info("Segmentation mask not available in YOLOv8 result.")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
