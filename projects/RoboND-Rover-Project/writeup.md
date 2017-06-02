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

[image1]: ./narrow.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

I add two functions for clor selection of obstacles and rock samples.
- the function for the obstacles is `color_obstacles`. It is actually the inverse of the function `color_thresh`. The threshold is the same as the road `(160, 160, 160)`. Since in numpy array, True is 1, False is 0, I directly define a variable `below_thresh` as the condition on the `img`, which makes the elements which below the threshold 1, above the threshold 0.

```python

def color_obstacles(img, rgb_thresh=(160, 160, 160)):
    below_thresh = (img[:,:,0] <= rgb_thresh[0]) \
                | (img[:,:,1] <= rgb_thresh[1]) \
                | (img[:,:,2] <= rgb_thresh[2])
    return below_thresh
```

- the function for the rocks is `color_rock`. The threshold is chosen as `(130, 100, 60)` where when elements in the Red layer and Green layer of img are greater than 130 and 100, respectively, and the Blue layer is less than 60, the corresponding `rock_img` elements are 1, otherwise 0. This condition gives the color the yellow, which is the color of rocks.

```python
def color_rock(img, rgb_thresh=(130, 100, 60)):
    rock_img = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    return rock_img
```
![alt text][image3]

#### 2. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result.
Basically I followed the TODO list in the `process_image()` function.
- I define the source and destination point, which we have already done in the class, so I just copy paste it here.
- Use `perspect_transform()` function to transform `img` into image `warped` based on the source and destination points. `warped` image is an image from the viewpoint right above the rover.
- Apply `color_thresh()` function to transform image `warped` and generate a binary image `terrain` that only shows the path. Apply `color_obstacles()` to `warped` to generate a binary image `obstacles` which only contains the obstacles. Apply `color_rock()` to `warped` to generate a binary image `rock` which only contains the target rocks.
- Use `rover_coords()` function to `terrain` and `obstacles` respectively to obtain the coordinates of path and obstacles in the rover-centric reference frame. For image `rock`, since rock is always with a height, when doing the `perspect_transform()`, in the `warped` image, the rock basically becomes a line (as shown in In [7] of the notebook). So I define a function `rock_pos()` to give the position from the pixel that is closest to the rover in the `warped` image, which is the pixel with the smallest `x` coordinate in the rover-centric reference frame.
```python
def rock_pos(x, y):
    if len(x) == 0 or len(y) == 0:
        return x, y
    else:
        idx = np.argmin(x)
        return x[idx], y[idx]
```

- Transform the rover-centric coordinates to world coordinates by using the `pix_to_world()` function. Here for the rock, since I used the `rock_pos()` function, the coordinate of the rock is just one point, in order to show it in the worldmap more explicitly, a function `rock_dense()`
is defined which adds eight more pixels around the rock pixel in the world map. In the map it would be a square of size 3 by 3.
Then the world map is updated as shown in the example.
```python
def rock_dense(x, y):
    if len([x]) == 0 or len([y]) == 0:
        return x, y
    else:
        return [x, x, x, x+1, x+1, x+1, x-1, x-1, x-1], [y, y-1, y+1, y, y-1, y+1, y, y-1, y+1]
```


### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.
`perception_step()` function is basically filled as in the `process_image()` function. Just change all entries to be class Rover related. One important addition is to update `Rover.vision_image`. I use
```python
Rover.vision_image[:,:,0] = obstacles * 200
Rover.vision_image[:,:,1] = rock * 200
Rover.vision_image[:,:,2] = terrain * 200
```
All binary image are multiplied by 200. This is because the pixels of binary images are only 1 or 0, in the RGB image, this makes no difference. With the multiplication, the pixels of the images becomes 200 or 0, which will give a distinguish among obstacles, rocks and paths.


There are several changes are made in the function `decision_step()`. First I define three variables `right_pix`, `left_pix` and `front_pix` in the function which count how many pixels are on the right-hand side, left-hand side and front, respectively.
```python
right_pix = np.sum(Rover.nav_angles <= 0)
left_pix = np.sum(Rover.nav_angles > 0)
front_pix = np.sum(Rover.nav_angles < 0.5) + np.sum(Rover.nav_angles > -0.5)
```

In order to traverse all of the maps, the strategy that I used is to make the rover prefer turn right more than turn left (prefer left than right also works). So I add the following code
```python
if front_pix < Rover.stop_forward + 50:
    Rover.throttle = 0
    #Rover.brake = Rover.brake_set/50
    if right_pix >= left_pix:
        Rover.steer = -15
    else:
        Rover.steer = 15
    # Rover.throttle = Rover.throttle_set
elif right_pix >= Rover.stop_forward:
    Rover.steer = np.clip(np.mean(Rover.nav_angles[Rover.nav_angles <= 0.5] * 180/np.pi), -15, 15)
else:
    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi)-4.5, -15, 15)
```
when the `Rover.mode` is `'forward'` and `len(Rover.nav_angles) >= Rover.stop_forward`. The first condition is to check if there is an obstacle in front of the rover, If it is true, then stop the throttle and turn right(left) if the right(left)-hand side is more clear than left(right)-hand side.

If there is no obstacle in front, then check if the right-hand side has enough road to go. If true, the rover chooses the direction which average the pixels with `Rover.nav_angles` not greater than 0.5. The small 0.5 on the left prevent the rover.steer is always set to -15.

If there is not enough road on the right-hand side, than average all pixel angles and -5. Because of the -5, the rover will always be closer to the right obstacles.

When the `Rover.mode` is `'stop'` and `Rover.vel` is smaller than 0.2, I change condition below to
```python
if right_pix < Rover.go_forward-200:
# if len(Rover.nav_angles) < Rover.go_forward:
    Rover.throttle = 0
    # Release the brake to allow turning
    Rover.brake = 0
    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
    Rover.steer = 15
```
which means that when the rover is stopped, the rover will turn left until it finds enough space on the right. This will prevent the rover stuck by stones.



#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

I use resolution 1024 * 768, and fantastic quality of image in the simulator. In the class `RoverState()`, I set
```python
self.throttle_set = 0.5
self.brake_set = 10
self.stop_forward = 50
self.go_forward = 600
self.max_vel = 2.5
```
Most of the time, the rover can navigate the map autonomouly, and it will traverse more than 98% of the map, and find at least 5 rocks in 5 or 6 mins. If set the lower `max_vel` and lower `throttle_set`, the performance is better. A better choice of `max_vel` would be 2.

The rover performs not very well when it meets several obstacles in the center of the map. Part of the reason is that the obstacles are not a solid... when the rover hit the obstacles, the camera somehow see through and gives wrong information. I think in the real case, the rover performs better :D In order to deal with the bug, one of the possible way is raise the threshold of the `front_pix` a bit higher. Such that the rover can make its decision before it hits the obstacles.

One important variable that I didn't use is `Rover.nav_dists`. By using it as condition, we may control the acceleration and velocity more accurately. 
