#!/usr/bin/python3
"""Demosaic and rectify a calibrated RELLIS camera at its native resolution."""
import copy
import json
from pathlib import Path

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image


class RellisCamera:
    def __init__(self):
        cal=json.loads(Path(rospy.get_param('~calibration_file')).read_text())
        self.size=(int(cal['width']),int(cal['height']))
        self.frame=cal['camera_frame']
        self.k=np.asarray(cal['K'],np.float64).reshape(3,3)
        self.d=np.asarray(cal['D'],np.float64)
        if cal['distortion_model']!='plumb_bob':raise ValueError('Expected plumb_bob calibration')
        self.mx,self.my=cv2.initUndistortRectifyMap(self.k,self.d,np.eye(3),self.k,self.size,cv2.CV_32FC1)
        self.bridge=CvBridge()
        topic=rospy.get_param('~output_topic','/uam2te/rellis/image_rect')
        self.pub=rospy.Publisher(topic,Image,queue_size=1)
        self.info_pub=rospy.Publisher(topic+'/camera_info',CameraInfo,queue_size=1)
        self.info=CameraInfo(width=self.size[0],height=self.size[1],distortion_model='plumb_bob')
        self.info.K=self.k.ravel().tolist();self.info.D=[0.]*len(self.d)
        self.info.R=np.eye(3).ravel().tolist()
        self.info.P=np.column_stack((self.k,np.zeros(3))).ravel().tolist()
        rospy.Subscriber('/pylon_camera_node/camera_info',CameraInfo,self.on_info,queue_size=1)
        rospy.Subscriber('/pylon_camera_node/image_raw',Image,self.on_image,queue_size=1,buff_size=32*1024*1024)

    def on_info(self,msg):
        if ((msg.width,msg.height)!=self.size or msg.header.frame_id!=self.frame or
            not np.allclose(msg.K,self.k.ravel(),rtol=0,atol=1e-6) or
            not np.allclose(msg.D,self.d,rtol=0,atol=1e-6)):
            rospy.signal_shutdown('RELLIS CameraInfo changed or differs from selected calibration')

    def on_image(self,msg):
        if (msg.width,msg.height)!=self.size or msg.header.frame_id!=self.frame:
            rospy.signal_shutdown('RELLIS image geometry differs from calibration');return
        # cv_bridge handles the ROS Bayer encoding, including RGGB->BGR.
        raw=self.bridge.imgmsg_to_cv2(msg,'bgr8')
        rect=cv2.remap(raw,self.mx,self.my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
        info=copy.deepcopy(self.info);info.header=copy.deepcopy(msg.header)
        image=self.bridge.cv2_to_imgmsg(rect,'bgr8');image.header=copy.deepcopy(msg.header)
        self.info_pub.publish(info);self.pub.publish(image)


if __name__=='__main__':
    rospy.init_node('rellis_camera_rectifier')
    RellisCamera()
    rospy.spin()
