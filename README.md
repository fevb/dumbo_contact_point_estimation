dumbo_contact_point_estimation
==============================

Implements and launches Dumbo specific components for contact point estimation.
Requires the following packages (can be found in https://github.com/kth-ros-pkg):

* [contact_point_estimation][1]
* [cart_traj_generators][2]
* [dumbo_cart_vel_controller][3]

[1] https://github.com/kth-ros-pkg/contact_point_estimation
[2] https://github.com/kth-ros-pkg/cart_traj_generators
[3] https://github.com/kth-ros-pkg/dumbo_cart_vel_controller

dumbo_surface_tracing_control_node
---------------------------------------------

Implements a surface tracing controller with a PI normal force regulator and a circular trajectory. 
Also publishes the twist of the force-torque sensor with respect to the base frame to the surface normal
estimator.