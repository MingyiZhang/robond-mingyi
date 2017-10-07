#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

# world = 1, 2, 3
WORLD = 3

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    # print(data_dict)
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # TODO: Statistical Outlier Filtering
    flt = cloud.make_statistical_outlier_filter()
    flt.set_mean_k(20)
    flt.set_std_dev_mul_thresh(0.3)
    cloud = flt.filter()

    # TODO: Voxel Grid Downsampling
    flt = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.005
    flt.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud = flt.filter()

    # TODO: PassThrough Filter
    flt = cloud.make_passthrough_filter()
    flt_axis = 'z'
    flt.set_filter_field_name(flt_axis)
    axis_min = 0.6
    axis_max = 1.5
    flt.set_filter_limits(axis_min, axis_max)
    cloud = flt.filter()

    flt = cloud.make_passthrough_filter()
    flt_axis = 'y'
    flt.set_filter_field_name(flt_axis)
    axis_min = -0.5
    axis_max = 0.5
    flt.set_filter_limits(axis_min, axis_max)
    cloud = flt.filter()

    # TODO: RANSAC Plane Segmentation
    seg = cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    cloud_table = cloud.extract(inliers, negative=False)
    cloud_objects = cloud.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)

    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(30)
    ec.set_MaxClusterSize(10000)
    tree = white_cloud.make_kdtree()
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cloud_cluster = pcl.PointCloud_PointXYZRGB()
    cloud_cluster.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_cluster = pcl_to_ros(cloud_cluster)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cloud_cluster)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)

        ros_cluster = pcl_to_ros(pcl_cluster)

        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .3
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    TEST_SCENE_NUM = Int32()
    TEST_SCENE_NUM.data = WORLD
    OBJECT_NAME = String()
    WHICH_ARM = String()
    PICK_POSE = Pose()
    PLACE_POSE = Pose()

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')

    # TODO: Parse parameters into individual variables
    # labels = []
    # centroids = [] # to be list of tuples (x, y, z)
    # for object in object_list:
    #     labels.append(object.label)
    #     points_arr = ros_to_pcl(object.cloud).to_array()
    #     centroids.append(np.mean(points_arr, axis=0)[:3])

    object_group_dict = {}
    for obj in object_list_param:
        object_group_dict[obj['name']] = obj['group']
    # print(object_group_dict)

    dropbox_dict = {}
    color_pos_dict = {}
    for box in dropbox_param:
        dropbox_dict[box['name']] = box['position']
        color_pos_dict[box['group']] = box['name']
    # print(dropbox_dict)
    # print(color_pos_dict)

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    dict_list = []
    for i, obj in enumerate(object_list):

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        OBJECT_NAME.data = str(obj.label)

        points_arr = ros_to_pcl(obj.cloud).to_array()
        PICK_POSE.position.x = np.asscalar(np.mean(points_arr, axis=0)[0])
        PICK_POSE.position.y = np.asscalar(np.mean(points_arr, axis=0)[1])
        PICK_POSE.position.z = np.asscalar(np.mean(points_arr, axis=0)[2])
        # print(PICK_POSE.position.x, PICK_POSE.position.y, PICK_POSE.position.z)

        PICK_POSE.orientation.x = 0
        PICK_POSE.orientation.y = 0
        PICK_POSE.orientation.z = 0
        PICK_POSE.orientation.w = 0

        # TODO: Create 'place_pose' for the object
        # TODO: Assign the arm to be used for pick_place
        WHICH_ARM.data = str(color_pos_dict[object_group_dict[obj.label]])

        PLACE_POSE.position.x = dropbox_dict[WHICH_ARM.data][0]
        PLACE_POSE.position.y = dropbox_dict[WHICH_ARM.data][1]
        PLACE_POSE.position.z = dropbox_dict[WHICH_ARM.data][2]

        PLACE_POSE.orientation.x = 0
        PLACE_POSE.orientation.y = 0
        PLACE_POSE.orientation.z = 0
        PLACE_POSE.orientation.w = 0

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        dict_list.append(make_yaml_dict(TEST_SCENE_NUM, WHICH_ARM, OBJECT_NAME, PICK_POSE, PLACE_POSE))
        # print(make_yaml_dict(TEST_SCENE_NUM, WHICH_ARM, OBJECT_NAME, PICK_POSE, PLACE_POSE))

        # Wait for 'pick_place_routine' service to come up
        # rospy.wait_for_service('pick_place_routine')
        #
        # try:
        #     pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
        #
        #     # TODO: Insert your message variables to be sent as a service request
        #     resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)
        #
        #     print ("Response: ",resp.success)
        #
        # except rospy.ServiceException, e:
        #     print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    yaml_filename = 'output_' + str(WORLD) + '.yaml'
    # print(dict_list)

    send_to_yaml(yaml_filename, dict_list)



if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('object_recognition', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size = 1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size = 1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size = 1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size = 1)

    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size = 1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size = 1)

    # TODO: Load Model From disk
    model = pickle.load(open('model_'+str(WORLD)+'.sav', 'rb'))
    # model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
