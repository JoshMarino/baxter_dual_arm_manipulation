Baxter Dual-arm Manipulation Optimization
=============================================

Josh Marino 
---------------------------------------------


#### Table of Contents ####
[Project Overview](#Project Overview)

[MoveIt!](#MoveIt!)

[Selecting a Pose for the Optimal Handoff](#Pose)

[Planning Trajectories to Optimal Handoff Location](#Trajectory)

[Attempts to Better Optimize Handoff Location and Trajectory](#Optimize)

[Future Work](#Future Work)

[Project Extensions](#Project Extensions)

[Torque Control](#Torque Control)

[Mass Balance](#Mass Balance)

[Joint_Trajectory_Controller](#Joint_Trajectory_Controller)



#### Project Overview  <a name="Project Overview"></a>
The intention of this project was to gain some experience in manipulation tasks with the Baxter research robot. The task at hand was to pass an object from one gripper to the next in minimal time, for any initial poses of each end-effector. In order to solve this complex task, it was broken down into three steps:

1) Minimization problem for finding handoff pose for each end-effector that are constrained to being a certain distance and rotation apart from one another
2) Using MoveIt! to generate paths for both end-effectors from initial pose to handoff pose, without being in collision
3) Performing the first and second steps numerous times in order to find the path that takes the minimal time


#### MoveIt! <a name="MoveIt!"></a>
moveit_commander
move_group 



#### Selecting a Pose for the Optimal Handoff  <a name="Pose"></a>




#### Planning Trajectories to Optimal Handoff Location  <a name="Trajectory"></a>




#### Attempts to Better Optimize Handoff Location and Trajectory  <a name="Optimize"></a>



#### Future Work  <a name="Future Work"></a>
