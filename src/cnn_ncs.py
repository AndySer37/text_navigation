#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_srvs.srv import Empty
from text_navigation.srv import *
import numpy as np
from os.path import expanduser
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import Rect, Rects, FSMState
from mvnc import mvncapi as mvnc
import time

np.set_printoptions(threshold=np.inf)
home = expanduser("~")
##/pbody/fsm_node/mode
class CNN_node():
    def __init__(self):
        self.image_sub = rospy.Subscriber("~ncs_image_rect", Image, self.img_cb,queue_size = 5)
        self.mode_sub = rospy.Subscriber("~mode", FSMState, self.fsm_mode)
        self.image_pub = rospy.Publisher('image_with_box', Image, queue_size=5)
        self.veh_name = rospy.get_param('~veh_name')
        #for img_cb
        self.bridge = CvBridge()
        self.cv_image = 0
        self.cv_img_crop = []

        #caffe params
        self.model = model = 'street_en_harvest'
        
        self.device_work = False
        mvnc.SetGlobalOption(mvnc.GlobalOption.LOG_LEVEL, 2)
        self.dim = (100, 32)        

        #counter & flag params\

        self.ncs_set = 0
        self.start = 0
        self.time = 0
        self.n = 1
        self.stop_line = 0  ### 0 
        self.deviceCheck()
    def deviceCheck(self):
        #check device is plugged in
        self.devices = mvnc.EnumerateDevices()
        if len(self.devices) == 0:
            self.device_work = False
            rospy.loginfo('NCS device not found')
	else:
            self.device_work = True
            rospy.loginfo('NCS device found')
            self.initialDevice()

    def initialDevice(self):
        # set the blob, label and graph
        self.device = mvnc.Device(self.devices[0])
        self.device.OpenDevice()
        network_blob = home + "/" + self.model + '.graph'
        
        ###for docker image
        #network_blob =  "/home/" + self.model + '.graph'
        #Load blob
        with open(network_blob, mode='rb') as f:
            blob = f.read()

        self.graph = self.device.AllocateGraph(blob)
        self.ncs_set = 1

    def fsm_mode(self, msg):
        print "FSM node callback"
        if msg.state == "INTERSECTION_CONTROL":
            self.stop_line = 1
            print "start text spotting"        
        elif msg.state == "JOYSTICK_CONTROL":
            self.stop_line = 0
            print "stop text spotting" 

    def img_cb(self, data):
        if self.stop_line == 0 or self.ncs_set == 0:
            return
        try:
            print "start detection"
            start = time.time()
            
            #for SR300 image  
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #for compressed image
            #np_arr = np.fromstring(data.data, np.uint8)
            #self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            img_gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            mser = cv2.MSER_create(20, 300, 20000, 0.25, 0.2, 200, 1.01, 0.003, 5)
            regions, boxes = mser.detectRegions(img_gray)
            hulls = [cv2.convexHull(p.reshape(-1, 1, 2)) for p in regions]
            #cv2.polylines(gray_img, hulls, 1, (255, 0, 0), 2)
            imgContours = self.cv_image
            contour_list = []

            point_list = [[0,0]]
            point_check = True

            for i, contour in enumerate(hulls):
                x,y,w,h = cv2.boundingRect(contour)

                if 2*h < w and h*w < 10000 and h*w > 1000:
                    point_check = True
                    for p in point_list:
                        #print p
                        if  p[0]-20 < x < p[0]+20 and p[1]-20 < y < p[1]+20:
                            point_check = False
                    if point_check == True:
                        point_list.append([x,y])
                        cv2.rectangle(imgContours,(x, y),(x+w, y+h),(0,255,0),3)
                        img_region = self.cv_image[y:y+h, x:x+w]
                        self.cv_img_crop.append(img_region)
            #publish image_with_box
            image_all_mser_image = Image()
            image_all_mser_image.header = rospy.Time.now
            image_all_mser_image = self.bridge.cv2_to_imgmsg(imgContours, "bgr8")
            self.image_pub.publish(image_all_mser_image)
            print "detection time:",time.time()-start

            #call prediction
            self.cnn()  

        except CvBridgeError as e:
            print(e)

    def cnn(self):
        if type(self.cv_img_crop) == []:
            print "No crop image receive"
            return
        print "strat prediction"
        i = 0

        #print len(self.cv_img_crop)

        for im in self.cv_img_crop:
            start = time.time()
            if im is None:
                break
            im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
            im = im.reshape(im.shape[0], im.shape[1], )
            im = cv2.resize(im, self.dim)         
            im = im.astype(np.float32)
            ###print "Prediction time:",time.time()- start
            self.graph.LoadTensor(im.astype(np.float16), 'user object')
            
            output, userobj = self.graph.GetResult()
            if i == 0:
                now = rospy.get_rostime().secs
                self.time += (now-self.start)
                #print self.n, self.time
                self.n += 1
            i += 1 
            #order = output.argsort()[::-1][:4]
            top1 = output.argmax()

            if output[top1] >= 0.9:
                print 'class: ',top1
                print output[top1]             

            
            if output[top1] >= 0.9:
                if top1 == 4 or top1 == 2:
                    turn_right = rospy.ServiceProxy('/' + self.veh_name + '/open_loop_intersection_control_node/turn_right', Empty)
                    turn = turn_right()
                    #topomap_action = rospy.ServiceProxy('topo_map_action', actions)
                    #action = actions()
                    #action.action = "R"
                    #resp = topomap_action(action)
                    #print "target node: ", resp.target_state
                elif top1 == 3:
                    turn_left = rospy.ServiceProxy('/' + self.veh_name + '/open_loop_intersection_control_node/turn_left', Empty)
                    turn = turn_left()
                    # topomap_action = rospy.ServiceProxy('topo_map_action', actions)
                    # action = actions()
                    # action.action = "L"
                    # resp = topomap_action(action)
                    # print "target node: ", resp.target_state
                elif top1 == 1 or top1 == 5:
                    turn_forward = rospy.ServiceProxy('/' + self.veh_name + '/open_loop_intersection_control_node/turn_forward', Empty)
                    turn = turn_forward()
                    # topomap_action = rospy.ServiceProxy('topo_map_action', actions)
                    # action = actions()
                    # action.action = "F"
                    # resp = topomap_action(action)
                    # print "target node: ", resp.target_state
                
                self.stop_line = 0
                break
        print "stop text spotting"
            
        self.cv_img_crop = []

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))  


def main(args):
    rospy.init_node('cnn_node', anonymous = True)
    ic = CNN_node()
    rospy.on_shutdown(ic.onShutdown)
    rospy.spin()



if __name__ == '__main__':
    main(sys.argv)
