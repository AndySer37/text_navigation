#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from text_navigation.srv import *
import numpy as np
from os.path import expanduser
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import Rect, Rects, FSMState
# added for workstation
caffe_root = expanduser("~")
sys.path.insert(0, caffe_root + '/caffe/python')
import caffe
import time

np.set_printoptions(threshold=np.inf)
home = expanduser("~")

class FCN_node():
    def __init__(self):
        self.node_name = "FCN"
        self.image_sub = rospy.Subscriber("/camera/color/image_rect_color", Image, self.img_cb)
        self.mode_sub = rospy.Subscriber("/pbody/fsm_node/mode", FSMState, self.fsm_mode)
        self.image_pub = rospy.Publisher('image_with_box', Image, queue_size=10)
        self.image_pred_pub = rospy.Publisher('image_with_prediction', Image, queue_size=10)

        #for img_cb
        self.bridge = CvBridge()
        self.cv_image = 0
        self.cv_img_crop = []
        self.cv_crop_region = []

        #caffe params
        self.model = model = 'street_en_harvest_fcn_49'
        self.caffe_root = home +'/caffe'
        sys.path.insert(0, self.caffe_root + 'python')
        caffe.set_mode_gpu()
        self.net_full_conv = caffe.Net(home+'/models/'+self.model+'.prototxt', home+'/models/'+self.model+'.caffemodel', caffe.TEST)
        
        #counter & flag params
        self.start = 0
        self.time = 0
        self.n = 1
        self.stop_line = 0

        #navigation map
        self.navi_map = []
        self.getParams()

    def getParams(self):

        c = rospy.get_param('~navigation_map')

        #self.loginfo('navigation map config: %s' % str(c))

        for i in range(len(c)):
            self.navi_map.append(['non', 'non'])

        for name, navigation in c.items():
            #print name, navigation
            [st_id, st_name] = name.split('_', 1 )
            #print st_id, st_name
            self.navi_map[int(st_id)][0] = st_name
            self.navi_map[int(st_id)][1] = navigation

        print self.navi_map

    def fsm_mode(self, msg):
        print "FSM node callback"
        if msg.state == "INTERSECTION_CONTROL":
            self.stop_line = 1
            print "start text spotting"        
        elif msg.state == "JOYSTICK_CONTROL":
            self.stop_line = 0
            print "stop text spotting" 

    def img_cb(self, data):
        if self.stop_line == 0:
            return
        try:
            print "start detection"
            caffe.set_mode_gpu()
            #self.start = data.header.stamp.secs
            #start = time.time()
            
            #for SR300 image  
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #for compressed image
            #np_arr = np.fromstring(data.data, np.uint8)
            #self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            #call prediction
            self.fcn()  

        except CvBridgeError as e:
            print(e)

    def fcn(self):
        print "strat prediction"
        start = time.time()
        img_pred = self.cv_image
        im = self.cv_image
        im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        
        im = im.reshape(im.shape[0], im.shape[1], 1)
        transformer = caffe.io.Transformer({'data': self.net_full_conv.blobs['data'].data.shape})

        transformer.set_transpose('data', (2,0,1))
        #transformer.set_raw_scale('data', 255.0)
        transformed_image = transformer.preprocess('data', im)
        transformed_image -= np.mean(transformed_image)

        self.net_full_conv.blobs['data'].data[...] = transformed_image
        out = self.net_full_conv.forward()
        print "Prediction time:",time.time()- start
        
        # filtering fcn result based on top1 probability per pixel
        max_index = out['prob'][0].argmax(axis=0)
        im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)

        # filtering fcn result based on top1 probability per pixel
        fcn_region = np.zeros((max_index.shape[0],max_index.shape[1]), dtype=float)
        for i in range(max_index.shape[0]):
            for j in range(max_index.shape[1]):
                if out['prob'][0][max_index[i][j]][i][j] >= 0.9:
                    fcn_region[i][j] = max_index[i][j]
                    print max_index[i][j]
                else:
                    fcn_region[i][j] = -1  
        
        # calculate the upsampling kernel
        fcn_input_region = np.full((self.net_full_conv.blobs['data'].data.shape[2],self.net_full_conv.blobs['data'].data.shape[3]),fill_value=-1, dtype=float)
        input_w_stride = int(self.net_full_conv.blobs['data'].data.shape[3] / ((max_index.shape[1]-1) * 1 + 13))
        input_h_stride = int(self.net_full_conv.blobs['data'].data.shape[2] / ((max_index.shape[0]-1) * 1 + 4))
        input_kernel_x = 13 * input_w_stride
        input_kernel_y = 4 * input_h_stride

        # upsampling fcn result to input dimension
        for i in range(fcn_region.shape[0]):
            for j in range(fcn_region.shape[1]):
                if(fcn_region[i][j] > 0):
                    #print fcn_region[i][j]
                    x_left =  0 + 8 * (j)
                    x_right = input_kernel_x + 8 * (j)
                    y_top =  0 + 8 * (i)
                    y_bottom = input_kernel_y + 8 * (i)
                    fcn_input_region[y_top:y_bottom, x_left:x_right] = fcn_region[i][j]
                               

        for i in range(out['prob'][0].shape[0]):
            roi = np.where(fcn_input_region == i)
            if len(roi[0]) == 0:
                continue
            roi_min_y = min(roi[0])
            roi_max_y = max(roi[0])
            roi_min_x = min(roi[1])
            roi_max_x = max(roi[1])
            print 'class ', i
            print roi_min_x, roi_min_y
            print roi_max_x, roi_max_y
        
            fcn_input_region = cv2.resize(fcn_input_region, (640,480))

            roi = np.where(fcn_input_region == i)
            roi_min_y = min(roi[0])
            roi_max_y = max(roi[0])
            roi_min_x = min(roi[1])
            roi_max_x = max(roi[1])
            print roi_min_x, roi_min_y
            print roi_max_x, roi_max_y
            cv2.rectangle(im, (roi_min_x, roi_min_y), (roi_max_x, roi_max_y),(0, 0,255),3)
            cv2.putText(im,str(self.navi_map[i][0]),(roi_min_x, roi_min_y),cv2.FONT_HERSHEY_COMPLEX, 1,(255,0,0),2)

        #cv2.rectangle(im, (roi_min_x, roi_min_y), (roi_max_x, roi_max_y),(1,0,0),3)


        image_all_pred_image = Image()
        image_all_pred_image.header = rospy.Time.now
        #fcn_region2 = np.uint8(fcn_input_region)
        #fcn_region2 = cv2.cvtColor(fcn_region2, cv2.COLOR_GRAY2BGR)
        #image_all_pred_image = self.bridge.cv2_to_imgmsg(fcn_region2, "bgr8")
        #fcn_region2 = np.uint8(fcn_input_region)

        
        #cv2.rectangle(im, (roi_min_x, roi_min_y), (roi_max_x, roi_max_y),(255,0,0),3)
        image_all_pred_image = self.bridge.cv2_to_imgmsg(im, "bgr8")
        self.image_pred_pub.publish(image_all_pred_image)
        #print 'pub'

			       

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name)) 

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))

def main(args):
    rospy.init_node('fcn_node', anonymous = True)        
    ic = FCN_node()
    rospy.on_shutdown(ic.onShutdown)
    rospy.spin() 

if __name__ == '__main__':
    main(sys.argv)
