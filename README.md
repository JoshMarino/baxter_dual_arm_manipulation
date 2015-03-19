Baxter Dual-arm Manipulation Optimization
=============================================

Josh Marino 
---------------------------------------------


#### Table of Contents ####
[Project Overview](#Project Overview)

[Selecting a Pose for the Optimal Handoff](#Pose)

[Planning Trajectories to Optimal Handoff Location](#Trajectory)

[Attempts to Better Optimize Handoff Location and Trajectory](#Optimize)

[Future Work](#Future Work)



#### Project Overview  <a name="Project Overview"></a>
The intention of this project was to gain some experience in manipulation tasks with the Baxter research robot. The task at hand was to pass an object from one gripper to the next in minimal time, for any initial poses of each end-effector. In order to solve this complex task, it was broken down into three steps:

1. Minimization problem for finding handoff pose for each end-effector that are constrained to being a certain distance and rotation apart from one another
2. Using MoveIt! to generate paths for both end-effectors from initial pose to handoff pose, without being in collision
3. Performing the first and second steps numerous times in order to find the path that takes the minimal time


#### Selecting a Pose for the Optimal Handoff  <a name="Pose"></a>
As mentioned in the [Project Overview](#Project Overview) section, the first step was solving a minimization problem for finding a handoff pose for each end-effector. Python's [scipy.optimize.minimize()](http://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html) function was used to solve this problem, using [Sequential Least Squares Programming, SLSQP](http://www.pyopt.org/reference/optimizers.slsqp.html) method. 

Rather than optimizing for an end-effector pose, the minimization problem was solved for the joint angles (q). Also, the pose used was a vector of six components, position in the x, y, z directions, and [XYZ Euler angles](http://en.wikipedia.org/wiki/Euler_angles), more commonly called roll, pitch, yaw. The objective function used for the minimization problem was the norm between the current end-effector pose and the corresponding end-effector pose for the set of q's found by the minimization, for each end-effector. Two function are called inside the objective function, (1) calculating the norm and (2) returning the pose of the end-effector with the current set of q's.

```p

# Minimization function to find a hand-off pose starting with initial right and left end-effector poses
minimization = lambda q: (norm(JS_to_P(q[0:7],'left'), P_left_current) + norm(JS_to_P(q[7:14],'right'), P_right_current))

```

The norm function used for this calculation was the square root of sum of squares between the corresponding end-effector pose and the current end-effector pose. Inside the norm function, another function, JS_to_P(q,'limb') is called. This uses KDLKinematics from [pykdl_utils](http://wiki.ros.org/pykdl_utils) to solve forward kinematics for the set of joint angles, while returning the six component vector pose. 

```p

# Takes configuration variables in joint space and returns the end-effector position and Euler XYZ angles
def JS_to_P(q,arm):

	robot = URDF.from_xml_file("/home/josh/catkin_ws/src/baxter_common/baxter_description/urdf/baxter.urdf")

	kdl_kin = KDLKinematics(robot, "base", str(arm)+"_gripper")

	q_correct_order = np.array([q[0], q[1], q[2], q[3], q[4], q[5], q[6]])

	T = kdl_kin.forward(q_correct_order)
	R = T[:3,:3]

	x = T[0,3]
	y = T[1,3]
	z = T[2,3]

	roll,pitch,yaw = euler_from_matrix(R, 'sxyz')

	P = np.array([[x],[y],[z],[roll],[pitch],[yaw]])

	return P

```

Also provided to the minimization function were rotational bounds for each joint, found in the URDF, and constraints to satisfy both end-effectors being a certain distance and rotation apart from one another. As for the distance apart from one another, 3 constraints were provided for the end-effectors being 0.0 m in the x-direction, 0.1 m in the y-direction, and 0.0 m in the z-direction. This would allow for a smooth handoff to occur by closing the gripper not initially holding the object, and then once closed, the other would release. Numerous attempts were made to determine the best constraints to ensure that the end-effector grippers would be 90 degrees away from one another. The best solution found was defining roll, pitch, yaw contraints from the rotation matrix that signifies the rotation from the right end-effector to the left end-effector. This rotation matrix was calculated by multiplying transpose(R_br)*(R_bl), where b stands for base frame, and r,l are for right and left gripper frames, respectively.


#### Planning Trajectories to Optimal Handoff Location  <a name="Trajectory"></a>




#### Attempts to Better Optimize Handoff Location and Trajectory  <a name="Optimize"></a>



#### Future Work  <a name="Future Work"></a>
