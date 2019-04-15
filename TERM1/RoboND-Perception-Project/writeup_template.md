# Project: Perception Pick & Place
[image1]: ./doc/train_svm.png
[image2]: ./doc/train_svm2.png
[image3]: ./doc/perception.png
[image4]: ./doc/perception2.png

## Content
 * Introduction
 * Required Steps for a Passing Submission
 * Extra Challenges
 * Perception Pipeline
 	* Filtering and RANSAC Plane Fitting 
 	* Pipeline including Clustering for Segmentation
 	* Feature Extraction and Model Training
 * Pick and Place Setup
 	* Extract YAML Files
 	* Pick and Place Action
 * Improvements
 
## Introduction
This project is about the perception of a robot equipped with a RGB-D camera. Therefore, it is necessary to use the pictures and point clouds given by the RGB-D camera to locate and indicate objects on a table lieing infront of the robot. To fullfill the projects requirements I've followed the  [Rubric](https://review.udacity.com/#!/rubrics/1067/view) points and the steps listet below for project passing.

## Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

## Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

### Perception Pipeline
#### Filtering and RANSAC Plane Fitting
The give point cloud from the RGB-D camera contains noise. Therefore, I used a statistical outlier filter to reduce the amout of noise and increase accuracy of the measured point cloud. To reduce the used data, I used a voxel grid filter, which reduces the used date and a pass through filter, which gives the pointcloud just in a specifc x, y and z range. To segment the table and the objects, I used the RANSAC plane fitting algorithm. This algorithm detects the table plane and seperates the point cloud into a table point cloud and an object pointcloud. This prepares the point cloud for clustering in the next step.

#### Pipeline including Clustering for Segmentation
 Furthermore, I used euclidean clustering to seperate the objects into distinct clusters and prepare them for object recognition. These clusters are used in the next part to extract surface normals histograms and a color histograms to predict the object labels.

#### Feature Extraction and Model Training

It is necessary to extract the object features (color histogramms and surface normals histograms) to train the SVM. I used the sensor stick setup from exercise 3 to extract the object features. Therefore, I copied the models from the project into the models folder of exercise 3 and changed the names in the `capture_features_pr.py` file in the `sensor_stick/scripts`directory. While cycling through every object, the script extracted features from `50` random orientations of every object. I used the generated `training_set_pr.sav` to train the svm with the `train_svm.py` file. Therefore, I used a linear kernel type with a C value of `0.1`. Figure 1 and 2 show the confusion matrizes of the trained model. And figure 3 shows the output of the object prediction in Rviz.

![alt text][image2]
*Figure 1 - Confusion Matrix*

![alt text][image1]
*Figure 2 - Normalized Confusion Matrix*

![alt text][image3]
*Figure 3 - Object Recognition in Wolrd 2*

For more details see `project_template.py`, `capture_features_pr.py`, `train_svm.py` and `features.py`in the `sensor_stick`directory.

### Pick and Place Setup

To pick the right object and place it into the right box, the robot needs further information from the object recognition pipeline. 

####  Extract YAML Files

Therefore, for all three tabletop setups (`test*.world`), I performed object recognition, calculated the centroid of every object to extract the pick pose, read in the respective pick list (`pick_list_*.yaml`) to know the place pose and to robot arm to use for the pick and place action. With all these information I constructed a ros message that would comprise a valid `PickPlace` request and output it to `.yaml` file (`output_*.yaml`). The YAML files contain all the mentioned information.

#### Pick and Place Action

To create a safe pick and place routine, I turned the robot by 360°. This helps to create a map and tell the robot where it can and where it can not go. The pick and place action uses the same ros massages that I created for the `output_*.yaml`files to feed the “pick_place_routine” rosservice with the given information. The `output_*.yaml` files can be found in the `pr2_robot/scripts`direction with the `model.sav`and the `training_set_pr.sav`.

For more details see `project_template.py`. 

### Improvements

The svm could be trained with another kernel or another C value to increase the robustness of the object recognition pipeline. Furthermore, I'd improve the logic of using the `pr2_move`function inside the `pcl_callback`function to increase the robustness of the the object recognition and just pick objects if they are perfectly identified. The calculation of the centroids (pick_pose) has to be improved to pick the objects perfectly. Right now, the robot misses most of the objects and leavesthem on the table. This could also be improved with a better filter pipeline. Furthermore, I couldn't get to run the statistical outlier filter which is probably the reason for the bad pickup action. Furthermore, I'd improve the collision avoidance point cloud which is fed into the path planning algorithm.