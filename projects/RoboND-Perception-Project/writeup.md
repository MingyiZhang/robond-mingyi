## Project: Perception Pick & Place

All realization of Exercises 1-3 and the project are in [here](https://github.com/MingyiZhang/robond-mingyi/blob/master/projects/RoboND-Perception-Project/pr2_robot/scripts/project_template.py).

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.
- Convert ROS message to PCL data
- Statistical Outlier Filter:
    - number of neighboring points: 20
    - threshold scale factor: 0.3
- Voxel Grid Downsampling
    - `LEAF_SIZE`: 0.005
- Passthrough over Z-Axis and Y-Axis
    - Z-Axis: [0.6, 1.5]
    - Y-Axis(not necessary): [-0.5, 0.5]
- RANSAC PLANE Filter
    - Maximum distance threshold: 0.01

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.
__Euclidean clustering__. After several attempts, the following parameter works fine
- Tolerances for distance threshold: 0.01
- Minimum cluster size: 30
- Maximum cluster size: 10000 (seems not really matter for a large number)

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.




Here is an example of how to include an image in your writeup.

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image!
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  
