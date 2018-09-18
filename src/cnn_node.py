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

class CNN_node():
    def __init__(self):
        self.node_name = "CNN"
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
        self.model = model = 'street_en_harvest'
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
            start = time.time()
            
            #for SR300 image  
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #for compressed image
            #np_arr = np.fromstring(data.data, np.uint8)
            #self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            img_gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            mser = cv2.MSER_create(25, 100, 250000, 0.25, 0.2, 200, 1.01, 0.003, 5)
            regions, _ = mser.detectRegions(img_gray)
            hulls = [cv2.convexHull(p.reshape(-1, 1, 2)) for p in regions]
            #cv2.polylines(gray_img, hulls, 1, (255, 0, 0), 2)
            imgContours = self.cv_image
            contour_list = []
            for i, contour in enumerate(hulls):
                x,y,w,h = cv2.boundingRect(contour)
                #repeat = self.checkContourRepeat(contour_list, x, y, w, h)
                #img_region = img_cv[y:y+h, x:x+w]      
                if 2*h < w and h*w < 10000 and h*w > 1000:
                    cv2.rectangle(imgContours,(x, y),(x+w, y+h),(0,255,0),3)
                    img_region = self.cv_image[y:y+h, x:x+w]
                    self.cv_img_crop.append(img_region)
                    self.cv_crop_region.append([x, y, w, h])
             
            #publish image_with_box
            image_all_mser_image = Image()
            image_all_mser_image.header = rospy.Time.now
            image_all_mser_image = self.bridge.cv2_to_imgmsg(imgContours, "bgr8")
            self.image_pub.publish(image_all_mser_image)
            print "detection time:",time.time()-start

            #call prediction
            #self.cnn()  

        except CvBridgeError as e:
            print(e)

    def cnn(self):
        if type(self.cv_img_crop) == []:
            print "No crop image receive"
            return
        print "strat prediction"
        img_pred = self.cv_image
        for i, im in enumerate(self.cv_img_crop):
            start = time.time()
            if im is None:
                break
            im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
            im = im.reshape(im.shape[0], im.shape[1], 1)

            transformer = caffe.io.Transformer({'data': self.net_full_conv.blobs['data'].data.shape})
            transformer.set_transpose('data', (2,0,1))
            #transformer.set_raw_scale('data', 255.0)
            self.net_full_conv.blobs['data'].reshape(1,1,32,100)
            transformed_image = transformer.preprocess('data', im)
            transformed_image -= np.mean(transformed_image)

            #make classification map by forward and print prediction indices at each location
            self.net_full_conv.blobs['data'].data[...] = transformed_image
            #self.net_full_conv.blobs['data'].data[...] = im
            #out = self.net_full_conv.forward(data=np.asarray(transformed_image))
            out = self.net_full_conv.forward()
            print "Prediction time:",time.time()- start
            
            top1 = out['prob'][0].argmax()
            x = self.cv_crop_region[i][0]
            y = self.cv_crop_region[i][1]
            w = self.cv_crop_region[i][2]
            h = self.cv_crop_region[i][3]          
            
            if out['prob'][0][top1] >= 0.9:
                print 'class: ',top1
                print out['prob'][0][top1][0][0]

                cv2.rectangle(img_pred,(x, y),(x+w, y+h),(0, 0,255),3)
                print self.navi_map[top1]
                cv2.putText(img_pred,str(self.navi_map[top1][0]),(x,y),cv2.FONT_HERSHEY_COMPLEX, 1,(255,0,0),2)
                
                if top1 == 4 or top1 == 2:
                    turn_right = rospy.ServiceProxy('/pbody/open_loop_intersection_control_node/turn_right', Empty)
                    turn = turn_right()
                    topomap_action = rospy.ServiceProxy('topo_map_action', actions)
                    action = actions()
                    action.action = "R"
                    resp = topomap_action(action)
                    print "target node: ", resp.target_state
                elif top1 == 23:
                    turn_left = rospy.ServiceProxy('/pbody/open_loop_intersection_control_node/turn_left', Empty)
                    turn = turn_left()
                    topomap_action = rospy.ServiceProxy('topo_map_action', actions)
                    action = actions()
                    action.action = "L"
                    resp = topomap_action(action)
                    print "target node: ", resp.target_state
                elif top1 == 14 or top1 == 22:
                    turn_forward = rospy.ServiceProxy('/pbody/open_loop_intersection_control_node/turn_forward', Empty)
                    turn = turn_forward()
                    topomap_action = rospy.ServiceProxy('topo_map_action', actions)
                    action = actions()
                    action.action = "F"
                    resp = topomap_action(action)
                    print "target node: ", resp.target_state
                self.stop_line = 0
                break
                print "stop text spotting"
                
        #publish image_with_box
        image_all_pred_image = Image()
        image_all_pred_image.header = rospy.Time.now
        image_all_pred_image = self.bridge.cv2_to_imgmsg(img_pred, "bgr8")
        self.image_pred_pub.publish(image_all_pred_image)
        self.cv_img_crop = []
        self.cv_crop_region = []
    
    def prediction_visualization(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name)) 


    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name)) 

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))

def main(args):
    rospy.init_node('cnn_node', anonymous = True)        
    ic = CNN_node()
    rospy.on_shutdown(ic.onShutdown)
    rospy.spin() 

if __name__ == '__main__':
    main(sys.argv)
