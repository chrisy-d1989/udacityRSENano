## Project: Search and Sample Return

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./calibration_images/color_thresh.png
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 


### Here I will consider the [rubric](https://review.udacity.com/#!/rubrics/916/view) points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
**Modifications of the notebook file**

* I've modified the `color_thresh` function with color thresholds for the navigable terrain, obstacles and rock samples. Furthermore I've used the `cv2.inRange`Function to create a binary image within the given thresholds. Three example pictures can be seen below.
![alt text][image1]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 
* I've defined source and destination points for the `perspect_transform` function to transform the incoming pictures
* I've used the `color_thresh` function to detect navigable terrain, obstacles and rock samples with different color thresholds defined in the `color_thresh` fucntion
* I've updated the `Rover.vision_image` with these binary pictures from the `color_thresh` function
* I've usaed the `rover_coords` function in combination with the binary pictures from the `color_thresh` function to convert map image pixel values to rover-centric coordinates. 
* With the `pix_to_world` function I've converted rover-centric pixels to world coordinates and used this to update the worldmap

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

I've modified the drive_rover.py, decision. py and the perception.py as followed.

**drive_rover.py:**

* I've extended the `__init__(self)` function with several variables as  for example `self.approaching_timer` to use these variables throughout the entire software stack.

**perception.py**

* I've defined source and destination points for the `perspect_transform` function to transform the incoming pictures
* I've used the `color_threh` function to detect navigable terrain, obstacles and rock samples with different color thresholds defined in the `color_thresh` fucntion
* I've updated the `Rover.vision_image` with the binary pictures from the `color_thresh` function
* I've used the `rover_coords` function in combination with the binary pictures from the `color_thresh` function to convert map image pixel values to rover-centric coordinates. 
* With the `pix_to_world` function I've converted rover-centric pixels to world coordinates and used this to update the worldmap, but only when roll and pitch angles are smaller than 0.5 ° or bigger than 359.5 ° to increase fidelity
* if the image contains more than 7 pixels of detected rock pixels, the `rock_located` flag will be set `True` to initiate sample approaching mode defined in `decision.py`
* With the `polar_to_coords` function I've calculated the angle and distance between the rover and the naviagble terrain or rock sample
* Record start position at the beginning of the simulation

**decision.py:**

* I've added the `Rover.mode`circling to detect if the Rover is driving in circles. If so, the Rover changes it's steering angle from +15 to -15 or the other way for 2 seconds and goes back to `forward`mode after that
*   I've added the `Rover.mode`stuck to detect, if the Rover is stuck and not moving althought throttel is at `0.2`. If so, the Rover turns itself for 3 seconds around it's own axis and goes back to `forward` mode after that
* I've added the `Rover.mode`approaching, to process the sample pick up if a rock sample is detected.
* In `approaching`mode:
	* I am calculating the distance `dist_to_rock`and the mean angle `mean_rock_angle` to the rock sample
	* If a rock sample is detected the rover slows down to 1 m/s 
	* If the `mean_rock_angle` is betwen +10° and -10° the rover approaches it with a steering angle between -15° and +15° and a throttle of 0.15 to get a smother approaching 
	* If  the `mean_rock_angle` is betwen +60° and -60° the rover stops and turns around its axis thill the `mean_rock_angle` is between -10° and 10°.
	* If `dist_to_rock < 1.5` the rover stops and the sample pickup gets initiated. After that the rover get back to `forward`mode
	* If sample approaching takes longer than 30 seconds, the process gets cancelled and the rover turns back to `forward`mode
* While in `forward`mode the rover checks if:
	* it's  `stuck`, with checking the velocity, throttel and time
	* it's `cicling` with checking the steering angle, velocity and time
	* there is navigable terrain. If there is, it turns throttel to 0.2 and if not, it sets the brake to 10 and turns
	* the steering angle is > 10° or < -10° to set throttel to 0 to increase fidelity
	* the `rock_located` flag is set to True to turn into `approaching`mode
* If every rock sample is located the rover stops, if its near to the starting position

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

The simulation results were made with 15-20 fps, a resolution of 1920:1080 and an Intel Corporation Sky Lake Integrated Graphics GPU. After 40 % of mapping and 128 seconds, the rover located 2 rock samples and collected 1 of it. The map had a fidelity of 66.4%. Afer 600 seconds, the rover mapped 80.5 % of the map with a fidelity of 54.2 %. It located 4 rock samples and collected 1 of it.
The rover can navigate safely throughout the environment and locate rock samples. Furthermore, the rover generates a map of visited areas with a fidelity of around 60 %. This depends strongly on the driving dynamics of the rover and the image processing step. To improve this, I'd either improve the driving of the rover to drive more smothly or the image processing step to exclude pixels which are farther away. Furthermore, the rover has no intelligence which places it has already visited and which not. To improve mapping speed I'd introduce a function which detects, if a location has already been visited. If the rock sample is close enough (<15m), it will approach it and try to pick it up. This works fine, if the rock sample is located on flat ground. However, if the rover's pitch angle is not ~0 it can't pickup the rock sample, because it has either a velocity bigger/smaller than 0 or throttles.  Furthermore, the logic should be increased if the rover has collected every sample, because right now it's stops randomly if it's crossing the starting area and it is not going straight back to the starting point.


