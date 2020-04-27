#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

import math
import sys
import numpy as np
from keras.models import load_model, model_from_json
from keras.utils.generic_utils import get_custom_objects
from keras import backend

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        #self.pose = None
        #self.waypoints = None
        #self.camera_image = None
        #self.lights = []

        self.pose_stamped = None
        self.waypoints_stamped = None
        self.caemra_image = None
        self.lights = None
        self.has_image = False
        self.light_classifier = TLClassifier()
        self.tf_listener = tf.TransformListener()
        self.prev_light_loc = None

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.lights_wp = []
        self.stoplines_wp = []

        self.camera_callback_count = 0

        self.simulated_detection = rospy.get_param('~simulated_detection', 1)
        self.tl_detection_interval_frames = rospy.get_param('~tl_detection_interval_frames', 10)

        config_string = rospy.get_param("/traffice_light_config")
        self.config = yaml.load(config_string)

        # Setup classifier
        rospy.loginfo("[TL_DETECTOR Loading TLClassifier model")
        self.light_classifier = TLClassifier()
        model = load_model(self.config['tl']['tl_classification_model'])
        resize_width = self.config['tl']['classifier_resize_width']
        resize_height = self.config['tl']['classifier_resize_height']

        self.light_classifier.setup_classifier(model, resize_width, resize_height)
        self.invlaid_class_number = 3

        # Setup detector
        rospy.loginfo("[TL_DETECTOR] Loading TLDetector model")
        custom_objects = {'dice_coef_loss': dice_coef_loss, 'dice_coef': dice_coef}
        
        self.detector_model = load_model(self.config['tl']['tl_detection_model'], custom_objects = custom_objects)
        self.detector_model._make_predict_function()
        self.resize_width = self.config['tl']['detector_resize_width']
        self.resize_height = self.config['tl']['detector_resize_height']
        self.resize_height_ratio = self.config['camera_info']['image_height'] / float(self.resize_height)
        self.resize_width_ratio = self.config['camera_info']['image_width'] / float(self.resize_width)
        self.middle_col = self.resize_width / 2
        self.is_carla = self.config['tl']['is_carla']
        self.projection_threshold = self.config['tl']['projection_threshold']
        self.projection_min = self.config['tl']['projection_min']
        self.color_mode = self.config['tl']['color_mode']

        
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()

        '''
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        '''

        rospy.spin()


    def pose_cb(self, msg):
        self.pose = msg


    def waypoints_cb(self, waypoints):
        #self.waypoints = waypoints
        if self.waypoints_stamped is not None:
            return
        
        self.waypoints_stamped = msg

        for i in range(len(self.waypoints_stamped.waypoints)):
            self.waypoints_stamped.waypoints[i].pose.header.frame_id = self.waypoints_stamped.header.frame_id

        self.calculate_traffic_light_waypoints()
    

    def traffic_cb(self, msg):
        #self.lights = msg.lights
        if self.simulated_detection > 0:
            self.lights = msg.lights
            self.calculate_traffic_light_waypoints()

            light_wp, state = self.process_traffic_lights()
            self.publish_upcoming_red_light(light_wp, state)
        else:
            if self.lights is not None:
                return
            
            self.lights = msg.lights
            self.calculate_traffic_light_waypoints()
            

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.camera_callback_count += 1

        if self.camera_callback_count < self.tl_detection_interval_frames:
            return

        self.camera_callback_count = 0


        # Original code
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1
        #Original code
        '''

        ###Added
        self.publish_upcoming_red_light(light_wp, state)


    def publish_upcoming_red_light(self, light_wp, state):
        if self.state != state:
            self.state_count = 0
            self.state = state
            
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        return 0

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)

        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
