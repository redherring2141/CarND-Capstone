#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
#import tensorflow as tf
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
VISIBLE_DISTANCE = 250
SMOOTHNESS = 1.0

def dice_coef(y_true, y_pred):
    y_true_f = backend.flatten(y_true)
    y_pred_f = backend.flatten(y_pred)
    intersection = backend.sum(y_true_f * y_pred_f)
    return (2.*intersection + SMOOTHNESS) / (backend.sum(y_true_f) + backend.sum(y_pred_f) + SMOOTHNESS)


def dice_coef_loss(y_true, y_pred):
    return -dice_coef(y_true, y_pred)

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

        config_string = rospy.get_param("/traffic_light_config")
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


    def waypoints_cb(self, msg):
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


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        #return 0

        if self.waypoints_stamped is None:
            return None
        
        dist_min = sys.maxsize
        wp_min = None

        for wp in range(len(self.waypoints_stamped.waypoints)):
            dist = self.dist_euclead(pose, self.waypoints_stamped.waypoints[wp].pose.pose)

            if dist < dist_min:
                dist_min = dist
                wp_min = wp
            
        return wp_min


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        labels = list(enumerate(['Red', 'Yellow', 'Green', 'None', 'None']))
        if self.simulated_detection > 0:
            if self.lights is None or light >= len(self.lights):
                rospy.loginfo("[TL_DETECTOR] simulated_detectionL: No TL is detected. None")    
                return TrafficLight.UNKNOWN
            state = self.lights[light].state
            rospy.loginfo("[TL_DETECTOR] simulated_detection: Nearest TL-state is: %s", labels[state][1])
            return state

        if(not self.has_image):
            self.prev_light_loc = None
            rospy.loginfo("[TL_DETECTOR] has_image is None: No TL is detected. None")
            return TrafficLight.UNKNOWN

        cv_img = self.bridge.imgmsg_to_cv2(self.camera_image, self.color_mode)
        tl_img = self.detect_traffic_light(cv_img)
        if tl_img is not None:
            #Get classification
            state = self.light_classifier.get_classification(tl_img)
            state = state if (state != self.invlaid_class_number) else TrafficLight.UNKNOWN
            rospy.loginfo("[TL_DETECTOR] Nearest TL-state is: %s", labels[state][1])
            return state
        else:
            rospy.loginfo("[TL_DETECTOR] tl_img is None: No TL is detected. None")
            return TrafficLight.UNKNOWN

            #return False


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        '''
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
        '''

        ###Added
        if self.pose_stamped is None or len(self.stoplines_wp) == 0:
            rospy.loginfo("[TL_DETECTOR] No TL is detected. None")
            return -1, TrafficLight.UNKNOWN

        # Find the closest traffic light if exists
        if light is None:
            rospy.loginfo("[TL_DETECTOR] No TL is detected. None")
            return -1, TrafficLight.UNKNOWN

        state = self.get_light_state(light)

        return self.stoplines_wp[light].state

        


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


    def extract_img(self, pred_img_mask, img):
        if np.max(pred_img_mask) < self.projection_min:
            return None

        row_projection = np.sum(pred_img_mask, axis=1)
        row_idx = np.argmax(row_projection)

        if np.max(row_projection) < self.projection_threshold:
            return None

        zero_row_idx = np.argwhere(row_projection <= self.projection_threshold)
        top_part = zero_row_idx[zero_row_idx < row_idx]
        top = np.max(top_part) if top_part.size > 0 else 0
        bottom_part = zero_row_idx[zero_row_idx > row_idx]
        bottom = np.min(bottom_part) if bottom_part.size > 0 else self.resize_height

        roi = pred_img_mask[top:bottom, :]
        col_projection = np.sum(roi, axis=0)

        if np.max(col_projection < self.projection_min):
            return None

        non_zero_col_idx = np.argwhere(col_projection > self.projection_min)

        idx_of_col_idx = np.argmin(np.abs(non_zero_col_idx - self.middle_col))
        col_idx = non_zero_col_idx[idx_of_col_idx][0]

        non_zero_col_idx = np.argwhere(col_projection == 0)
        left_side = non_zero_col_idx[zero_col_idx < col_idx]
        left = np.max(left_side) if left_side.dize > 0 else 0
        right_side = zero_col_idx[zero_col_idx > col_idx]
        right = np,min(right_side) if right_side.size > 0 else self.resize_width
        return image[int(top*self.resize_height_ratio):int(bottom*self.resize_height_ratio),
                     int(left*self.resize_width_ratio):int(right*self.resize_width_ratio)]

    
    def detect_tl(self, cv_img):
        resize_img = cv2.cvtColor(cv2.resize(cv_img, (self.resize_width, self.resize_height)), cv2.COLOR_RGB2GRAY)
        resize_img = resize_img[..., np.newaxis]
        if self.is_carla:
            avg = np.mean(resize_img)
            std = np.std(resize_img)
            resize_img -= avg
            resize_img /= std

        img_mask = self.detector_model.predict(resize_img[None,:,:,:], batch_size=1)[0]
        img_mask = (img_mask[:,:,0]*255).astype(np.uint8)
        return self.extract_img(img_mask, cv_img)


    def dist_euclead(self, pos1, pos2):
        distance = (pos1.position.x-pos2.position.x)**2 + (pos1.position.y-pos2.position.y)**2
        return distance


    def transform_to_car_frame(self, pose_stamped):
        try:
            self.tf_listener.waitForTransform("base_link", "world", rospy.Time(0), rospy.Duration(0.02))
            transformed_pose_stamped = self.tf_listener.transformPose("base_link", pose_stamped)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            transformed_pose_stamped = None
            rospy.logwarn("Failed to transform pose")

        return transformed_pose_stamped


    def get_closest_stopline_pose(self, pose):
        stop_line_positions = self.config['stop_line_positions']
        dist_min = sys.maxsize
        stop_line_min = None

        for stop_line_position in stop_line_positions:
            stop_line_pose = Pose()
            stop_line_pose.position.x = stop_line_position[0]
            stop_line_pose.position.y = stop_line_position[1]
            stop_line_pose.position.z = 0.0

            dist = self.dist_euclead(pose, stop_line_pose)

            if dist < dist_min:
                dist_min = dist
                stop_line_min = stop_line_pose

        return stop_line_min


    def calculate_traffic_light_waypoints(self):
        if self.waypoints_stamped is not None and self.lights is not None and len(self.lights_wp) == 0:
            for i in range(len(self.lights)):
                stopline = self.get_closest_stopline_pose(self.lights[i].pose.pose)
                self.stoplines_wp.append(self.get_closest_waypoint(stopline))
                self.lights_wp.append(self.get_closest_waypoint(self.lights[i].pose.pose))


    def get_closest_visible_traffic_light(self, pose):
        if self.waypoints_stamped is None or self.lights is None or len(self.lights_wp) == 0:
            return None

        num_lights = len(self.lights_wp)

        dist_min = sys.maxsize
        light_min = None

        for light in range(num_lights):
            dist = self.dist_euclead(pose, self.waypoints_stamped.waypoints[self.lights_wp[light]].pose.pose)

            if dist < dist_min:
                dist_min = dist
                light_min = light

        transformed_waypoint = self.transform_to_car_frame(self.waypoints_stamped.waypoints[self.lights_wp[light_min]].pose)

        if transformed_waypoint is not None and transformed_waypoint.pose.position.x <= 0.0:
            light_min += 1

        if light_min >= num_lights:
            light_min -= num_lights

        dist_euclead = self.dist_euclead(pose, self.waypoints_stamped.waypoints[self.lights_wp[light_min]].pose)

        if dist_euclead > (VISIBLE_DISTANCE ** 2):
            return None

        return light_min


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
