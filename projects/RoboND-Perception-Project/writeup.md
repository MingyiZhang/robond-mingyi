## Project: Perception Pick & Place

All realization of Exercises 1-3 and the project are in [here](https://github.com/MingyiZhang/robond-mingyi/blob/master/projects/RoboND-Perception-Project/pr2_robot/scripts/project_template.py).

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.
- Convert ROS message to PCL data
- Statistical Outlier Filter:
    - number of neighboring points: __20__
    - threshold scale factor: __0.3__
- Voxel Grid Downsampling
    - `LEAF_SIZE`: __0.005__
- Passthrough over Z-Axis and Y-Axis
    - Z-Axis: __[0.6, 1.5]__
    - Y-Axis(not necessary): __[-0.5, 0.5]__
- RANSAC PLANE Filter
    - Maximum distance threshold: __0.01__

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.
__Euclidean clustering__. After several attempts, the following parameter works fine
- Tolerances for distance threshold: __0.01__
- Minimum cluster size: __30__
- Maximum cluster size: __10000__ (seems not really matter for a large number)

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
__Support Vector Machine__. Use the following setups.
- images per object: __50__
- Bins: __32__
- HSV: __Yes__

We can get a [model](https://github.com/MingyiZhang/robond-mingyi/tree/master/projects/RoboND-Perception-Project/models) (The `model_3.sav` file) which can recognize objects.

And here is the confusion matrices after the training

![alt text](https://github.com/MingyiZhang/robond-mingyi/blob/master/projects/RoboND-Perception-Project/imgs/confusion_matrix_3.png)
![alt text](https://github.com/MingyiZhang/robond-mingyi/blob/master/projects/RoboND-Perception-Project/imgs/confusion_matrix_norm_3.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

Follow the instruction, the PR2 can recognise the objects and perform Pick and Place.

The code basically follows the instruction. I introduced three dictionaries `object_group_dict`, `color_pos_dict` and `dropbox_dict` to map object to group(box color), to map group(box color) to box(left or right) and to map box(left or right) to drop place position, respectively.
```python
object_group_dict = {}
for obj in object_list_param:
    object_group_dict[obj['name']] = obj['group']

dropbox_dict = {}
color_pos_dict = {}
for box in dropbox_param:
    dropbox_dict[box['name']] = box['position']
    color_pos_dict[box['group']] = box['name']    
```

The final results are:
- World 1: 100%, [YAML file](https://github.com/MingyiZhang/robond-mingyi/blob/master/projects/RoboND-Perception-Project/outputs/output_1.yaml)

    ![alt text](https://github.com/MingyiZhang/robond-mingyi/blob/master/projects/RoboND-Perception-Project/imgs/world_1.png)
- World 2: 100%, [YAML file](https://github.com/MingyiZhang/robond-mingyi/blob/master/projects/RoboND-Perception-Project/outputs/output_2.yaml)

    ![alt text](https://github.com/MingyiZhang/robond-mingyi/blob/master/projects/RoboND-Perception-Project/imgs/world_2.png)
- World 3: 100%, [YAML file](https://github.com/MingyiZhang/robond-mingyi/blob/master/projects/RoboND-Perception-Project/outputs/output_3.yaml)

    ![alt text](https://github.com/MingyiZhang/robond-mingyi/blob/master/projects/RoboND-Perception-Project/imgs/world_3.png)

If I have more time, I would like to perform the challenge.
