#!/usr/bin/env python
import rospy
from keras.models import load_model
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from math import pow, sqrt
import tf
import cv2
import yaml
import sys

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.detector_model = None
        self.resized_width = -1
        self.resized_height = -1
        self.resized_height_ratio = -1
        self.resized_width_ratio = -1
        self.lights = []
        self.use_ground_truth = sys.argv[1].lower() == 'true'
        self.distance_to_tl_threshold = 50.0

        rospy.loginfo("[TL_DETECTOR] Use GT for TL: %s", self.use_ground_truth)

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
        sub4 = rospy.Subscriber('camera_info', CameraInfo, self.camera_info_cb)

        if not self.use_ground_truth:
            sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()

        # Classifier Setup
        self.light_classifier = TLClassifier()
        model = load_model(self.config['tl']['tl_classification_model'])
        resize_width = self.config['tl']['classifier_resize_width']
        resize_height = self.config['tl']['classifier_resize_height']
        self.light_classifier.setup_classifier(model, resize_width, resize_height)

        #Detector setup
        self.detector_model = load_model(self.config['tl']['tl_detection_model'])
        self.resized_width = self.config['tl']['detector_resize_width']
        self.resize_height = self.config['tl']['detector_resize_height']
        self.is_carla = self.config['tl']['is_carla']
        self.projection_threshold = self.config['tl']['projection_threshold']

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

        # If ground truth data from tl is enabled.
        if self.use_ground_truth:
            light_wp, state = self.process_traffic_lights()
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.upcoming_red_light_pub.publish(Int32(light_wp))

    def camera_info_cb(self, msg):
        if self.resize_height != -1 and self.resize_width != -1:
            if self.resized_height_ratio == -1 and self.resized_width_ratio == -1:
                self.resized_height_ratio = msg.height/self.resize_height
                self.resized_width_ratio = msg.width/self.resize_width

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
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

    def dist_to_point(self, pose, wp_pose):
        x_squared = pow((pose.position.x - wp_pose.position.x), 2)
        y_squared = pow((pose.position.y - wp_pose.position.y), 2)
        dist = sqrt(x_squared + y_squared)
        return dist



    def get_closest_waypoint(self, pose, waypoints):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
            waypoints : points where to look for closest one

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        min_dist = float("inf")
        closest_wp_idx = -1

        if not waypoints:
            rospy.logwarn("[TL_DETECTOR] No waypoints given.")
        else:

            for idx, wp in enumerate(waypoints):
                dist = self.dist_to_point(pose, wp.pose.pose)
                if(dist < min_dist):
                    min_dist = dist
                    closest_wp_idx = idx
        return closest_wp_idx

    def extract_image(pred_image_mask, image):
        #if not (np.any(pred_image_mask[:,:] > 220)):
        #    return None

        column_projection = np.sum(pred_image_mask, axis = 0)

        if (np.max(column_projection) < self.projection_threshold):
            return None

        non_zero_column_index = np.argwhere(column_projection > projection_threshold)

        if non_zero_column_index.size == 0:
            return None

        index_of_column_index =  np.argmin(np.abs(non_zero_column_index - middle_col))
        column_index = non_zero_column_index[index_of_column_index][0]

        zero_colum_indexes = np.argwhere(column_projection == 0)
        left = np.max(zero_colum_indexes[zero_colum_indexes < column_index])
        right = np.min(zero_colum_indexes[zero_colum_indexes > column_index])

        #roi = pred_image_mask[:,left:right]

        row_projection = np.sum(pred_image_mask, axis = 1)
        row_index =  np.argmax(row_projection)

        zero_row_indexes = np.argwhere(row_projection == 0)
        top = np.max(zero_row_indexes[zero_row_indexes < row_index])
        bottom = np.min(zero_row_indexes[zero_row_indexes > row_index])

        return image[int(top*self.resized_height_ratio):int(bottom*self.resized_height_ratio), int(left*self.resized_width_ratio):int(right*self.resized_width_ratio)] 

    def detect_traffic_light(self, cv_image):
        if self.resized_width_ratio == -1 or self.resized_height_ratio == -1:
            return None

        resized_image = cv2.cvtColor(cv2.resize(cv_image, (self.resize_width, self.resize_height)), cv2.COLOR_RGB2GRAY)
        if self.is_carla:
            mean = np.mean(resized_image) # mean for data centering
            std = np.std(resized_image) # std for data normalization

            resized_image -= mean
            resized_image /= std

        image_mask = self.detector_model.predict(resized_image)
        return extract_image(image_mask, cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if self.pose is not None:
            tl_id = self.get_closest_waypoint(self.pose.pose, self.lights)
            rospy.logdebug("[TL_DETECTOR] Closest TL id: %s.", tl_id)
            if (tl_id >= 0):

                tl = self.lights[tl_id]
                stop_line = self.config['stop_line_positions'][tl_id]

                # Convert stop_line to pseudo pose object for easier handling
                stop_line_pose = Pose()
                stop_line_pose.position.x = stop_line[0]
                stop_line_pose.position.y = stop_line[1]

                wp_id = self.get_closest_waypoint(stop_line_pose, self.waypoints)
                rospy.logdebug("[TL_DETECTOR] Closest WP id: %s.", wp_id)

                if (wp_id == -1):
                    rospy.logdebug("[TL_DETECTOR] Unable to determine valid TL id.")
                    return -1, TrafficLight.UNKNOWN

                closest_wp = self.waypoints[wp_id]
                waypoint_dist = self.dist_to_point(closest_wp.pose.pose, tl.pose.pose)
                car_dist = self.dist_to_point(self.pose.pose, stop_line_pose)
                if self.use_ground_truth:
                    state = tl.state
                    rospy.loginfo("[TL_DETECTOR] Using ground-truth information. Nearest TL-state is: %s", state)
                elif(car_dist<self.distance_to_tl_threshold):
                    cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, self.config['tl']['color_mode'])
                    tl_image = self.detect_traffic_light(cv_image)
                    if tl_image != None:
                        state = self.light_classifier.get_classification(tl_image)
                        rospy.loginfo("[TL_DETECTOR] Nearest TL-state is: %s", state)
                    else:
                        rospy.loginfo("[TL_DETECTOR] No TL is detected")
                else:
                    rospy.loginfo("[TL_DETECTOR] Next TL too far yet.")
                    state = TrafficLight.UNKNOWN

                rospy.logdebug("[TL_DETECTOR] Upcoming TL ID: %s, State: %s, Distance: %s. ", tl_id, state, car_dist)
                rospy.logdebug("[TL_DETECTOR] Upcoming TL stop line position: x: %s | y: %s.", stop_line[0],
                              stop_line[1])
                return wp_id, state

            else:
                rospy.logwarn("[TL_DETECTOR] No trafic light found!")
                return -1, TrafficLight.UNKNOWN
        else:
            rospy.logwarn("[TL_DETECTOR] No EGO position available!")
            return -1, TrafficLight.UNKNOWN
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
