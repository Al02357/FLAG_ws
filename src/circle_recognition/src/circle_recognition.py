#!/usr/bin/env python
# !coding=utf-8
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from matplotlib import pyplot as plt
import math
import datetime
from geometry_msgs.msg import PoseStamped
import time 
import math


circle_pose_pub = None

cam0_path = '/home/intelnuc/catkin_ws_SLAM/src/circle_recognition/src/images/'  # 已经建立好的存储cam0 文件的目录
cam1_path = '/home/intelnuc/catkin_ws_SLAM/src/circle_recognition/src/'

fx=383.0
fy=383.0
cx=323.0
cy=238.0

r=0.35

K=np.array([[fx,0 ,cx],
            [0 ,fy,cy],
            [0 ,0 ,1 ]])



def callback(data):
    # define picture to_down' coefficient of ratio
    scaling_factor = 0.5
    global count, bridge
    count = count + 1
    if count == 1:
        count = 0
        img = bridge.imgmsg_to_cv2(data, "8UC1")
        timestr = "%.6f" % data.header.stamp.to_sec()
        # %.6f表示小数点后带有6位，可根据精确度需要修改；
        image_name = timestr + ".jpg"  # 图像命名：时间戳.jpg
        # cv2.imwrite(cam0_path + image_name, cv_img)  #保存；

        # img=cv2.blur(img,(1,1))
        # imgray=cv2.Canny(img,600,100,3)#Canny边缘检测，参数可更改
        imgray= cv2.Canny(img, 100, 20, 3)  # Canny边缘检测，参数可更改
        #asd = cv2.Canny(img, 100, 10, 3)  # Canny边缘检测，参数可更改
        #print(type(asd))
        #return 
        cv2.imshow("Canny",imgray)
        # cv2.waitKey(0)
        ret, thresh = cv2.threshold(imgray, 127, 255, cv2.THRESH_BINARY)
        #contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,
        #                                       cv2.CHAIN_APPROX_SIMPLE)  # contours为轮廓集，可以计算轮廓的长度、面积等
        _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)  # contours为轮廓集，可以计算轮廓的长度、面积等

        #print("ellipses:")
        # x,y,a,b
        Ellipses = []
        for cnt in contours:
            if len(cnt) > 50:
                S1 = cv2.contourArea(cnt)
                # ell[0][0]:x    ell[0][1]:y
                # ell[1][0]:a    ell[1][1]:b

                ell = cv2.fitEllipse(cnt)
                S2 = math.pi * ell[1][0] * ell[1][1]
                if S1 < 100 or (S1 / S2) < 0.2:
                    continue
                # if :#面积比例，可以更改，根据数据集。。。
                #    img = cv2.ellipse(img, ell, (0, 255, 0), 2)
                #    print(str(S1) + "    " + str(S2)+"   "+str(ell[0][0])+"   "+str(ell[0][1]))
                # print(str(S1) + "    " + str(S2) + "   " + str(ell[0][0]) + "   " + str(ell[0][1]))

                ok = 1
                for ell_for_sure in Ellipses:
                    # if np.linalg.norm(np.array(ell[0])-np.array(ell_for_sure[0]))>20:
                    #    ok = 0
                    #    break
                    if np.linalg.norm(np.array(ell[0]) - np.array(ell_for_sure[0])) < 20 and \
                            ell[1][0] / ell_for_sure[1][0] < 1.05 and \
                            ell[1][0] / ell_for_sure[1][0] > 0.95:
                        ok = 0
                        break
                    if np.linalg.norm(np.array(ell[0]) - np.array(ell_for_sure[0])) < 20 and \
                            ell[1][1] / ell_for_sure[1][1] < 1.05 and \
                            ell[1][1] / ell_for_sure[1][1] > 0.95:
                        ok = 0
                        break
                if ok == 1:
                    Ellipses.append(ell)
                    # img = cv2.ellipse(img, ell, (0, 255, 0), 2)
                    #print("    s    " + str(S1) + "    x   " + str(ell[0][0]) + "    y   " + str(
                    #    ell[0][1]) + "    a   " + str(ell[1][0]) + "    b   " + str(ell[1][1]))
                #print(" ")

        for i in range(len(Ellipses)):
            for j in range(len(Ellipses)):
                if i >= j:
                    continue
                ell1 = Ellipses[i]
                ell2 = Ellipses[j]

                same = 1
                if np.linalg.norm(np.array(ell1[0]) - np.array(ell2[0])) < 10 and \
                        (ell1[1][0] / ell2[1][0] > 1.05 or ell1[1][0] / ell2[1][0] < 0.95) and \
                        (ell1[1][1] / ell2[1][1] > 1.05 or ell1[1][1] / ell2[1][1] < 0.95) and \
                        (ell1[1][0] / ell2[1][0]) / (ell1[1][1] / ell2[1][1]) > 0.80 and \
                        (ell1[1][0] / ell2[1][0]) / (ell1[1][1] / ell2[1][1]) < 1.20:
                    a = max(ell1[1][0], ell2[1][0])

                    px = (ell1[0][0] - cx) / fx
                    py = (ell1[0][1] - cy) / fy
                    d = r * fx / a
                    Px = px * d
                    Py = py * d
                    Pz = d

                    img = cv2.ellipse(img, ell1, (0, 255, 0), 2)
                    img = cv2.ellipse(img, ell2, (0, 255, 0), 2)
                    cv2.circle(img, (int(ell1[0][0]), int(ell1[0][1])), 2, (0, 0, 255))

                    msg = PoseStamped()
                    msg.header = data.header
                    msg.pose.position.x = Px
                    msg.pose.position.y = Py
                    msg.pose.position.z = Pz

                    global circle_pose_pub
                    circle_pose_pub.publish(msg)


                    print("get one circle:", "    x:    ", Px, "    y:    ", Py, "    z:    ", Pz)
                    print("")

                    break



        cv2.imshow("frame", img)
        cv2.waitKey(3)






    else:
        pass


def displayWebcam():
    rospy.init_node('webcam_display', anonymous=True)

    global circle_pose_pub
    circle_pose_pub = rospy.Publisher('/circle_pose', PoseStamped, queue_size=100)

    # make a video_object and init the video object
    global count, bridge
    count = 0
    bridge = CvBridge()
    rospy.Subscriber('/D435i/infra1/image_raw', Image, callback)
    rospy.spin()


if __name__ == '__main__':
    displayWebcam()

