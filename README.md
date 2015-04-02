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

![successful_handoff](successful_iteration.png)


#### Project Overview  <a name="Project Overview"></a>
The intention of this project was to gain some experience in manipulation tasks with the Baxter research robot. The task at hand was to pass an object from one gripper to the next in minimal time, for any initial poses of each end-effector. In order to solve this complex task, it was broken down into three steps:

1. Minimization problem for finding handoff pose for each end-effector, which are constrained to being a certain distance and rotation apart from one another
2. Using MoveIt! to generate paths for both end-effectors from initial pose to handoff pose, without being in collision
3. Performing the first and second steps numerous times in order to find the path that takes the minimal time


#### Selecting a Pose for the Optimal Handoff  <a name="Pose"></a>
As mentioned in the Project Overview section, the first step was solving a minimization problem for finding a handoff pose for each end-effector. Python's [scipy.optimize.minimize()](http://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html) function was used to solve this problem, using [Sequential Least Squares Programming, SLSQP](http://www.pyopt.org/reference/optimizers.slsqp.html) method. 

Rather than optimizing directly for an end-effector pose, the minimization problem was solved in the domain of joint angles (q). Also, the pose used was a vector of six components, position in the x, y, z directions, and [XYZ Euler angles](http://en.wikipedia.org/wiki/Euler_angles), more commonly called roll, pitch, yaw. The objective function used for the minimization problem was the norm between the current end-effector pose and the corresponding end-effector pose for the set of q's found by the minimization, for each end-effector. Two functions are called inside the objective function, (1) calculating the norm and (2) returning the pose of the end-effector with the current set of q's.

```p

# Minimization function to find a hand-off pose starting with initial right and left end-effector poses
minimization = lambda q: (norm(JS_to_P(q[0:7],'left'), P_left_current) + norm(JS_to_P(q[7:14],'right'), P_right_current))

```

The norm function used for this calculation was the square root of sum of squares between the corresponding end-effector pose and the current end-effector pose. Inside the norm function, another function, JS_to_P(q, 'limb') was called. This uses KDLKinematics from [pykdl_utils](http://wiki.ros.org/pykdl_utils) to solve forward kinematics for the set of joint angles, while returning the six component vector pose. 

```p

# Takes configuration variables in joint space and returns the end-effector position and Euler XYZ angles
def JS_to_P(q, arm):

	robot = URDF.from_xml_file("~/catkin_ws/src/baxter_common/baxter_description/urdf/baxter.urdf")

	kdl_kin = KDLKinematics(robot, "base", str(arm)+"_gripper")

	T = kdl_kin.forward(q)
	R = T[:3,:3]

	x = T[0,3]
	y = T[1,3]
	z = T[2,3]

	roll,pitch,yaw = euler_from_matrix(R, 'sxyz')

	P = np.array([[x],[y],[z],[roll],[pitch],[yaw]])

	return P

```

Also provided to the minimization function were rotational bounds for each joint, found in the URDF, and constraints to satisfy both end-effectors being a certain distance and rotation apart from one another. As for the distance apart from one another, 3 constraints were provided for the end-effectors being 0.0 m in the x-direction, 0.1 m in the y-direction, and 0.0 m in the z-direction. This would allow for a smooth handoff to occur by closing the gripper not initially holding the object, and then once closed, the other gripper would release. 

Numerous attempts were made to determine the best constraints to ensure that the end-effector grippers would be rotated 90 degrees away from one another. The best solution found was defining roll, pitch, yaw constraints from the rotation matrix that signified the rotation from the right end-effector to the left end-effector. This rotation matrix was calculated by

> R_rl =  transpose(R_br)*(R_bl)

where b stands for base frame, and r, l are for right and left gripper frames, respectively.


#### Planning Trajectories to Optimal Handoff Location  <a name="Trajectory"></a>
After finding a set of joint angles that minimized the objective function while satisfying the constraints, collision-free paths had to be generated to move the end-effectors. [MoveIt!](http://moveit.ros.org/baxter-research-robot/) was used to generate the collision-free paths by setting the [move_group](https://github.com/davetcoleman/moveit_commander/blob/hydro-devel/src/moveit_commander/move_group.py) joint value goal targets to the joint angles found from the minimization function. In order to ensure collision-free paths, the move_group 'both_arms' from the [MoveGroupCommander](http://docs.ros.org/indigo/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html) was utilized. This allowed planning to occur for both end-effectors simultaneously, and collision-free planning was implemented.

The following tutorials were beneficial to help understand MoveIt! with Baxter:

1. [Rethink Robotics Baxter MoveIt Tutorial](http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial#Tutorial)
2. [Move Group Interface Python](http://docs.ros.org/hydro/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html)


#### Attempts to Better Optimize Handoff Location and Trajectory  <a name="Optimize"></a>
In order to get closer to finding the minimal time handoff, numerous handoff locations were found by changing the initial guess in the minimization problem. Also, for each handoff location, MoveIt! planned multiple paths to the goal state. The same, exact path would probabilistically not be found twice due to the probabilistic planners MoveIt! uses, such as [RRT](http://en.wikipedia.org/wiki/Rapidly_exploring_random_tree). Allowing MoveIt! to generate multiple paths to the same goal state would increase the likelihood of finding the minimal time trajectory. After each path was generated, the trajectory and the corresponding time it took was saved. After a certain amount of initial guesses were used, and the corresponding number of paths planned by MoveIt! completed, the trajectory with the least amount of time was executed.

Initially, the minimization problem was running slower than expected. A few changes were made to the minimization parameters for finding a solution in the least amount of time and iterations. First, the constraints were reduced to as few as possible, which included six, three for handoff separation distance and another three for the rotation between the two end-effectors. Next, the jacobian (gradient) of the objective function was passed as a parameter instead of being estimated numerically. Last, the tolerance for terminiation was set to be as large as possible, such that oscillations were not occuring around the solution. A tolerance of 0.1 was determined best after reducing the constraints and passing in the jacobian function. After these parameters were added, the minimization problem typically was solved in under 30 iterations and roughly 1-2 seconds. 

However, once the minimization problem was solved, the joint angle solution caused Baxter's limbs to be in collision for the handoff poses more often than I would have liked. To overcome this problem, another constraint was added which set the x,y position of the right end-effector halfway between the initial x,y positions of the end effector. This worked very nicely and trajectories were able to be planned in MoveIt! most of the time.

TBD: currently using 5 minimization and 10 MoveIt! paths



#### Future Work  <a name="Future Work"></a>
A possible improvement would be in the path planning portion using MoveIt! Instead of running OMPL (open motion planning library) in MoveIt!, a different planning library could be used. CHOMP is a novel method for continuous path refinement that uses covariant gradient techniques to improve the quality of sampled trajectories [N. Ratliff, M. Zucker, J. Bagnell, and S. Srinivasa. CHOMP: Gradient optimization techniques for efficient motion planning. In IEEE ICRA, 2009]. This allows for planning multiple paths while finding the best path from all of them. The best path is found by examining all planned paths and is guaranteed to find a path that is no worse than the best of all planned paths.
