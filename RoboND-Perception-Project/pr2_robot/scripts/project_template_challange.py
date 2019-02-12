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
import pcl
import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


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
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    #Convert ROS msg to PCL data
    cloud_pcl = ros_to_pcl(pcl_msg)
    
    ### find bug for outlier filtering
    # Statistical outlier filtering
    #outlier_filter = cloud_pcl.make_statistical_outlier_filter()
    # Set the number of neighboring points to analyze for any given point
    #outlier_filter.set_mean_k(20)
    # Any point with a mean distance larger than global will be considered out
    #outlier_filter.set_std_dev_mul_thresh(0.1)
    #cloud_filtered = outlier_filter.filter()
    
    # Voxel Grid Downsampling
    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud_pcl.make_voxel_grid_filter()
    # Choose a voxel (also known as leaf) size
    LEAF_SIZE = 0.01   
    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # Call the filter function to obtain the resultant downsampled point cloud  
    cloud_filtered = vox.filter()
    
    # PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    passed = passthrough.filter()
    # Limiting on the Y axis too to avoid having the bins recognized as snacks
    passthrough = passed.make_passthrough_filter()
    # Assign axis and range to the passthrough filter object.
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = - 0.45
    axis_max = 0.45
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = passthrough.filter()

    # RANSAC plane segmentation
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()
    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    # Max distance for a point to be considered fitting the model
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()
    # Extract inliers and outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    
    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(2500)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
             color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
	
    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)  
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)
  
    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    
    # create dict for avoid collision list
    avoid_dic = {} 
    # Grab the points for the cluster
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list) 
        ros_cluster = pcl_to_ros(pcl_cluster)
        # Compute the associated feature vector
        normed_color_features = compute_color_histograms(ros_cluster, using_hsv=True)
        normed_norm_features = compute_normal_histograms(get_normals(ros_cluster))
        feature = np.array([normed_color_features,normed_norm_features]) 
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))
	# create avoid_dic	
	avoid_dic[label] = [index, pts_list]
	
	# Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)
    
    #Publish Collision Avoidance PCL
    collision_avoid_table_pub.publish(ros_cloud_table)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        ros_object_name = pr2_mover(detected_objects, cluster_cloud, avoid_dic)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list, cluster_indices, avoid_dic):

    # Initialize variables
    ros_object_name = String()
    test_scene_num = Int32()
    pick_pose = Pose()
    arm_name = String()
    place_pose = Pose()	
    dict_list = []
    labels = []
    count = 0

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_list_param = rospy.get_param('/dropbox')
   
    # Rotate PR2 in place to capture side tables for the collision map
    #pub_j1 = rospy.Publisher('/pr2/world_joint_controller/command', Float64, queue_size=10)
    #pub_j1.publish(np.pi*2)     # Rotate the PR2 counter-clockwise 360 deg

    # Loop through the pick list
    for object_param in object_list_param:
	    # load yaml files params    	
	    object_name = object_param['name']
    	    object_group = object_param['group']
	    for object_idx, object_det in enumerate(object_list):
		
		    # check if object fits object in pickup list        	
		    if object_name != object_det.label:
		        # Skip this object if it doesn't match the object from the pickup list
		        continue		

		    labels.append(object_det.label)
		    
		    #define test scene and object name			
		    test_scene_num.data = 2 
		    ros_object_name.data = object_name
		    
		    # Get the PointCloud for a given object and obtain it's centroid
		    points_arr = ros_to_pcl(object_det.cloud).to_array()
		    centroid = np.mean(points_arr, axis=0)[:3]
            	    centroid_ros = [np.asscalar(x) for x in centroid]		
		    # define pick_pose for robot arm
		    pick_pose.position.x = centroid_ros[0]
		    pick_pose.position.y = centroid_ros[1]
		    pick_pose.position.z = centroid_ros[2]
		    pick_pose.orientation.x = 0.0
		    pick_pose.orientation.y = 0.0
		    pick_pose.orientation.z = 0.0
		    pick_pose.orientation.w = 0.0

		    # Create 'place_pose' for the object and define robot arm name
		    for dropbox_params in dropbox_list_param:
		        if dropbox_params['group'] == object_group:
		            place_pose.position.x = dropbox_params['position'][0] + (count * 0.01)
		            place_pose.position.y = dropbox_params['position'][1] + (count * 0.01)
		            place_pose.position.z = dropbox_params['position'][2] + (count * 0.01)
		            place_pose.orientation.x = 0.0
		            place_pose.orientation.y = 0.0
		            place_pose.orientation.z = 0.0
		            place_pose.orientation.w = 0.0	 			
			    # define robot arm name for pick and place operation
		            arm_name.data = dropbox_params['name']						

			    break	
	    	    # count moved objects for adjusting place pose
	    	    count += 1 

		    # delete object from collision avoidance pcl
		    for label, pts_list in avoid_dic.iteritems():
		    	if label != ros_object_name:
 	                    continue
			if(cluster_indices[pts_list[0]] == pts_list[1]):
        	            avoid_list = cluster_indices.pop(pts_list[0])
		        avoid_cloud = pcl.PointCloud_PointXYZRGB()
    		        #create new pcl from avoid_list
			avoid_cloud.from_list(avoid_list)
		        ros_avoid_cloud = pcl_to_ros(avoid_cloud)	
			collision_avoid_objects_pub.publish(ros_avoid_cloud)
		    
		    # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
		    #for i in range(0, len(object_list_param)):
	 	    # Populate various ROS messages
		    yaml_dict = make_yaml_dict(test_scene_num, arm_name, ros_object_name, pick_pose, place_pose)
		    dict_list.append(yaml_dict)
		    
		    # Wait for 'pick_place_routine' service to come up
		    rospy.wait_for_service('pick_place_routine')
		    try:
		        pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

		        # Insert your message variables to be sent as a service request
		        resp = pick_place_routine(test_scene_num,ros_object_name, arm_name, pick_pose, place_pose)

		        print ("Response: ",resp.success)

		    except rospy.ServiceException, e:
		        print "Service call failed: %s"%e

		    # Remove the object from object_list to indicate it was picked up
		    del object_list[object_idx]
		    
		    # Stop looking through the other identified objects
		    break
    
    # Output your request parameters into output yaml file
    send_to_yaml('output_{}.yaml'.format(test_scene_num.data), dict_list)
    print("successfully saved yaml output_{}.yaml".format(test_scene_num.data))

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # Create Subscriber
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detect_objects", DetectedObjectsArray, queue_size=1)
    
    # publisher for collision avoidance
    collision_avoid_table_pub = rospy.Publisher("/pr2/3D_map/points", PointCloud2, queue_size=1)
    collision_avoid_objects_pub = rospy.Publisher("/pr2/3D_map/points", PointCloud2, queue_size=1)
    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():   
	    rospy.spin()
