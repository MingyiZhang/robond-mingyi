## Project: Kinematics Pick & Place
---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[img_kin]: ./misc_images/kinematics_demo.png
[img_DH]: ./misc_images/joint_frame.png
[img_ik]: ./misc_images/ik.png



### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Run command `roslaunch kuka_arm forward_kinematics.launch` and the screenshot is

![alt text][img_kin]

The following figure shows the basic structure of Kuka KR210 robot.

![alt text][img_DH]

Eight frames associate to the base point, six joints and the gripper. The DH parameter are assigned by using the [algorithm](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/87c52cd9-09ba-4414-bc30-24ae18277d24/concepts/2f59c902-9c32-4b26-9e52-5e495ec14dba) in the course. Some of the DH parameter are shown in the figure.

One can derive the DH parameter through the demo, or from the file `kr210.urdf.xacro`. First of all, let us read the initial state (relative positions and orientations) of the joints through the demo,

| Joint |   X   | Y |    Z   | Roll-Pitch-Yaw |
|:-----:|:-----:|:-:|:------:|:--------------:|
|   1   |   0   | 0 |  0.33  |        0       |
|   2   |  0.35 | 0 |  0.42  |        0       |
|   3   |   0   | 0 |  1.25  |        0       |
|   4   |  0.96 | 0 | -0.054 |        0       |
|   5   |  0.54 | 0 |    0   |        0       |
|   6   | 0.193 | 0 |    0   |        0       |
|   G   |  0.11 | 0 |    0   |        0       |

Then we can derive the DH parameter

| i | $\alpha_{i-1}$ | $a_{i-1}$ | $d_i$ | $\theta_i$ |
|:-:|:-------------:|:---------:|:-----:|:---------:|
| 1 |       0       |     0     |  0.75 |     q1    |
| 2 |     -$\pi$/2     |    0.35   |   0   | q2 - $\pi$/2 |
| 3 |       0       |    1.25   |   0   |     q3    |
| 4 |     -$\pi$/2     |   -0.054   |  1.5  |     q4    |
| 5 |      $\pi$/2     |     0     |   0   |     q5    |
| 6 |     -$\pi$/2     |     0     |   0   |     q6    |
| G |       0       |     0     | 0.303 |     0     |

where qi, i= 1, 2, ..., 6 are independent variables of Kuka KR210 robot. So the degrees of freedom of the robot is 6. Given a set of qi will capture a unique state of the robot. The $-\pi/2$ deficit between q2 is because axis X1 and X2 are orthogonal in the initial state.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The individual transformation from frame i-1 to i by using the DH parameter is given as
$$T^{i-1}_i = R_X(\alpha_{i-1})\cdot D_X(a_{i-1})\cdot R_Z(\theta_i)\cdot D_Z(d_i)$$
where $R_X$ and $R_Z$ are rotations around axis X and Z, respectively, while $D_X$ and $D_Z$ are translations around axis X and Z, respectively. Practically, $R$ and $D$ are all homogeneous transformations, i.e. $4\times 4$ matrices.
$$
R = \begin{pmatrix}
r_{3\times 3} & 0_{3\times 1} \\
0 & 1
\end{pmatrix}, \quad
D = \begin{pmatrix}
I_{3\times 3} & p_{3\times 1} \\
0 & 1
\end{pmatrix}
$$
where $r$ is a 3d rotation matrix, $I$ is the identity matrix and $p$ is a 3-vector.

Then the homogeneous transform between base_link and gripper_link is given as
$$T^{0}_G = T^0_1\cdot T^1_2 \cdot~\cdots~\cdot T^6_G$$
A vector $V_{OG}$ from the origin $O$ to the gripper can be obtained by using $T^0_G$ to transform the gripper pose in the gripper frame $V_G = (0,0,0,1)^T$:
$$V_{OG} = T^0_G\cdot V_G$$



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The KUKA KR210 robot can be decomposed into two parts. The part with the first three joints (joint 1, 2 and 3) capture mainly the position kinematics of the robot, while the other part with the rest joints (joint 4, 5 and 6), called the spherical wrist, capture mainly the orientation kinematics of the robot. The center of the spherical wrist is called the wrist center (WC). It sits at the joint 5 and coincides to the origins of frame 4, 5 and 6.

In order to obtain q1, q2, ..., q6 from the pose of the gripper, we follow the steps below:

__step 1__: Split the pose of the gripper to frame 0 into two parts.

Since the WC is siting at the joint 5, the position of the gripper to frame 6 is given as
$$
V_{6G} = (0,0,d_G,1)^T
$$
The homogeneous transform $T^0_6$ from frame 0 to frame 6 is given as
$$
T^0_6 =
\begin{pmatrix}
r^0_6 & p_{\text{wc}} \\
0 & 1
\end{pmatrix}
$$
where $p_{\text{wc}}$ is the position of the WC to frame 0.
The position of the gripper to frame 0 is then given as
$$
V_{OG} = T^0_6\cdot V_{6G} = \left(d_G ~r_6^0\cdot \hat{z} + p_{\text{wc}}, ~1\right)^T
$$
where $\hat{z}\equiv (0,0,1)^T$.

__step 2__: obtain q1, q2, q3 such that WC's pose is $p_{\text{wc}}\equiv (p_x, p_y, p_z)$.

q1 is quite easy to get
$$
q_1 = \text{atan2}(p_y, p_x)
$$

We then move our reference frame to frame 1. The pose of WC in the new frame is then
$$
(p_{\text{wc}1}, ~1)^T \equiv (p_{x1}, 0,~ p_{z1},~ 1)^T  \equiv [T_1^0]^{-1}\cdot (p_{\text{wc}}, ~1)^T
$$
The system in frame 1 is shown in the following figure

![alt text][img_ik]

Let us get q3 first:
$$
q_{34} = \text{atan2}(d_4, a_3)
$$
$$
k_{34} = \sqrt{d_4^2 + a_3^2}
$$
$$
k_{24} = \sqrt{(p_{x1}-a_1)^2 + p_{z1}}
$$
Then
$$
\cos(q_{24}) = \frac{a_2^2 + k_{34}^2 - k_{24}^2}{2 a_2 k_{34}}
$$
$$
q_{24} = \text{atan2}\left(\sqrt{1-\cos^2(q_{24})}, ~\cos(q_{24})\right)
$$
Then
$$
q_3 = - (q_{24} - q_{34})
$$
Here we assume that $q_{24}$ from $k_{34}$ to $a_2$ clock-wise is always not bigger than $\pi$. The minus sign in front of the above equation is because q3 is negative if the arm rotates anti-clock-wise from its initial state in our convention.

Then $q_2$:
$$
\cos(q_{23}) = \frac{a_2^2 + k_{24}^2 - k_{34}^2}{2a_2 k_{24}}
$$
$$
q_{23} = \text{atan2}\left(\sqrt{1-\cos^2(q_{23})}, ~\cos(q_{23})\right)
$$
$$
q_{21} = \text{atan2}(p_{z1},~ p_{x1}-a_1)
$$
Then
$$
q_2 = \frac{\pi}{2} - q_{23} - q_{21}
$$

__step 3__: 









And here's where you can draw out and show your math for the derivation of your theta angles.

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]
