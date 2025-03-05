import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# R_H_low = 0
# R_S_low = 0
# R_V_low = 249
# R_H_high = 25
# R_S_high = 255
# R_V_high = 255

R_H_low = 0
R_S_low = 0
R_V_low = 236
R_H_high = 360
R_S_high = 23
R_V_high = 255

# 採樣間距
W_sampling_1 = 305
W_sampling_2 = 270
W_sampling_3 = 235
W_sampling_4 = 200



class Lane_detection(Node):

    def __init__(self):
        super().__init__('Lane_detection')
        self.subscription = self.create_subscription(
            Image,
            '/image/image_raw',
            self.right_line,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Int64, 'topic', 10)
        self.cv_bridge=CvBridge()
    # def listener_callback(self, msg):
    #     self.get_logger().info('I heard: "%s"' % msg.data)




    '''
        右循線
    '''
    def right_line(self, msg, kernel_size=25, low_threshold=10, high_threshold=20, close_size=5):

        # 將ROS Image轉換成OpenCV格式
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 左右線極限X值(需重置)
        R_min_300 = 640
        R_min_240 = 640
        R_min_180 = 640
        R_min_140 = 640

        # 影像預處理
        # img = copy(img)
        img = cv2.resize(img,(640,360))
        # img = cv2.GaussianBlur(img, (11, 11), 0)
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        # 右線遮罩
        lower_R = np.array([R_H_low,R_S_low,R_V_low])
        upper_R = np.array([R_H_high,R_S_high,R_V_high])
        mask_R = cv2.inRange(hsv,lower_R,upper_R)



        # 右線運算
        # 右線運算 - Canny邊緣運算
        blur_gray = cv2.GaussianBlur(mask_R,(kernel_size, kernel_size), 0)
        canny_img = cv2.Canny(blur_gray, low_threshold, high_threshold)

        # 右線運算 - 閉運算(解緩Canny斷線問題)
        kernel = np.ones((close_size,close_size),np.uint8)
        gradient = cv2.morphologyEx(canny_img, cv2.MORPH_GRADIENT, kernel)

        # 右線運算 - 霍夫變換
        lines = cv2.HoughLinesP(gradient,1,np.pi/180,8,5,2)
        # print("error")
        if type(lines) == np.ndarray:
            for line in lines:
                x1,y1,x2,y2 = line[0]
                if ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_1:
                    # cv2.line(img,(x1,y1),(x2,y2),(255,0,0),1)
                    if ((x1+x2)/2)<R_min_300:
                        R_min_300 = int((x1+x2)/2)
                elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_2:
                    # cv2.line(img,(x1,y1),(x2,y2),(0,255,0),1)
                    if ((x1+x2)/2)<R_min_240:
                        R_min_240 = int((x1+x2)/2)
                elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_3:
                    # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                    if ((x1+x2)/2)<R_min_180:
                        R_min_180 = int((x1+x2)/2)
                elif ((x1+x2)/2)>350 and ((y1+y2)/2)>W_sampling_4:
                    # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)
                    if ((x1+x2)/2)<R_min_140:
                        R_min_140 = int((x1+x2)/2)
        else:
            print("error")
            pass



        # cv2.rectangle(img, (L_min_300, W_sampling_1), (R_min_300, 360), (255,0,0), 0) 
        # cv2.rectangle(img, (L_min_240, W_sampling_2), (R_min_240, W_sampling_1), (0,255,0), 0) 
        # cv2.rectangle(img, (L_min_180, W_sampling_3), (R_min_180, W_sampling_2), (0,0,255), 0)
        # cv2.rectangle(img, (L_min_140, W_sampling_4), (R_min_140, W_sampling_3), (0,255,255), 0) 

        pts = np.array([[R_min_300,(360+W_sampling_1)/2], [R_min_240,(W_sampling_1+W_sampling_2)/2], [R_min_180,(W_sampling_2+W_sampling_3)/2],[R_min_140,(W_sampling_3+W_sampling_4)/2]], np.int32)
        pts = pts.reshape((-1, 1, 2))
        img = cv2.polylines(img, [pts], False,(255,200,0),3)

        # 計算結果(車頭偏左負號)
        R_min = ((R_min_300+R_min_240+R_min_180+R_min_140)/4)-320
        target_line = int(R_min-265)
        print(-target_line)
        
        # target_line=int64(target_line)
        
        pub_msg=Int64()
        pub_msg.data=-target_line
        self.publisher_.publish(pub_msg)
        # 輸出原圖&成果
        # cv2.imshow("img", img)
        # cv2.imshow("mask_R", mask_R)
        # cv2.waitKey(1)

        # return -target_line, img


def main(args=None):
    rclpy.init(args=args)

    lane_detection = Lane_detection()

    rclpy.spin(lane_detection)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lane_detection.destroy_node()
    rclpy.shutdown()