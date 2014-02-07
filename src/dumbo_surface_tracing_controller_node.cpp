/*
 *  dumbo_surface_tracing_controller_node.cpp
 *
 *
 *  Created on: Feb 7, 2014
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2014, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <contact_point_estimation/SurfaceTracingController.h>
#include <dumbo_cart_vel_controller/DumboCartVelController.h>
#include <cart_traj_generators/CircleTrajGenerator.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>
#include <boost/bind.hpp>

class DumboSurfaceTracingControllerNode : DumboCartVelController
{

public:

	ros::Publisher topicPub_twist_ft_sensor_;
	ros::Subscriber topicSub_ft_compensated_;

	/// declaration of service servers
	ros::ServiceServer srvServer_Start_;
	ros::ServiceServer srvServer_Stop_;

	DumboSurfaceTracingControllerNode() : DumboCartVelController()
	{
		m_run_controller = false;

		topicSub_CommandTwist_.shutdown();

		topicPub_twist_ft_sensor_ = n_.advertise<geometry_msgs::TwistStamped>("twist_ft_sensor", 1);
		topicSub_ft_compensated_ = n_.subscribe("ft_compensated", 1,
				&DumboSurfaceTracingControllerNode::topicCallback_ft_compensated, this);

		srvServer_Start_ = n_.advertiseService("start", &DumboSurfaceTracingControllerNode::srvCallback_Start, this);
		srvServer_Stop_ = n_.advertiseService("stop", &DumboSurfaceTracingControllerNode::srvCallback_Stop, this);

		m_surface_tracing_controller = new SurfaceTracingController();
		m_cart_traj_generator = NULL;

		// gets ROS parameters for the surface tracing controller
		// and sets the parameters in the controller object
		configureController();

		// gets ROS parameters for the trajectory generator
		// and sets the parameters in the controller object
		configureTrajectoryGenerator();

	}

	~DumboSurfaceTracingControllerNode()
	{
		delete m_surface_tracing_controller;
		delete m_cart_traj_generator;
	}

	void configureController()
	{

		// get the normal force compensation gains
		double alpha_p;

		if (n_.hasParam("controller/alpha_p"))
		{
			n_.getParam("controller/alpha_p", alpha_p);
		}

		else
		{
			ROS_ERROR("Parameter controller/alpha_p not set, shutting down node...");
			n_.shutdown();
			return;
		}

		double alpha_i;

		if (n_.hasParam("controller/alpha_i"))
		{
			n_.getParam("controller/alpha_i", alpha_i);
		}

		else
		{
			ROS_ERROR("Parameter controller/alpha_i not set, shutting down node...");
			n_.shutdown();
			return;
		}


		// get the trajectory position control gain
		double alpha;

		if (n_.hasParam("controller/alpha"))
		{
			n_.getParam("controller/alpha", alpha);
		}

		else
		{
			ROS_ERROR("Parameter controller/alpha not set, shutting down node...");
			n_.shutdown();
			return;
		}


		// get the normal force set point
		double f_d;

		if (n_.hasParam("controller/f_d"))
		{
			n_.getParam("controller/f_d", f_d);
		}

		else
		{
			ROS_ERROR("Parameter controller/f_d not set, shutting down node...");
			n_.shutdown();
			return;
		}

		// get the control frequency
		double control_freq;

		if (n_.hasParam("controller/control_freq"))
		{
			n_.getParam("controller/control_freq", control_freq);
		}

		else
		{
			ROS_ERROR("Parameter controller/control_freq not set, shutting down node...");
			n_.shutdown();
			return;
		}

		m_surface_tracing_controller->setNormalForceCompensationGains(alpha_p, alpha_i);
		m_surface_tracing_controller->setTrajectoryPosControlGain(alpha);
		m_surface_tracing_controller->setDesiredNormalForce(f_d);
		m_surface_tracing_controller->setControlFrequency(control_freq);
	}

	void configureTrajectoryGenerator()
	{
		std::string trajectory_type;

		if (n_.hasParam("trajectory_generator/trajectory_type"))
		{
			n_.getParam("trajectory_generator/trajectory_type", trajectory_type);
		}

		else
		{
			ROS_ERROR("Parameter trajectory_generator/trajectory_type not set, shutting down node...");
			n_.shutdown();
			return;
		}

		if(trajectory_type=="circle")
		{

			CircleTrajGenerator *circle_traj_generator = new CircleTrajGenerator();
			m_cart_traj_generator = circle_traj_generator;

			double radius;

			if (n_.hasParam("trajectory_generator/radius"))
			{
				n_.getParam("trajectory_generator/radius", radius);
			}

			else
			{
				ROS_ERROR("Parameter trajectory_generator/radius not set, shutting down node...");
				n_.shutdown();
				return;
			}


			double period;

			if (n_.hasParam("trajectory_generator/period"))
			{
				n_.getParam("trajectory_generator/period", period);
			}

			else
			{
				ROS_ERROR("Parameter trajectory_generator/period not set, shutting down node...");
				n_.shutdown();
				return;
			}


			double duration;

			if (n_.hasParam("trajectory_generator/duration"))
			{
				n_.getParam("trajectory_generator/duration", duration);
			}

			else
			{
				ROS_ERROR("Parameter trajectory_generator/duration not set, shutting down node...");
				n_.shutdown();
				return;
			}

			circle_traj_generator->setRadius(radius);
			circle_traj_generator->setPeriod(period);
			circle_traj_generator->setDuration(duration);
		}

		else
		{
			ROS_ERROR("Invalid trajectory_type parameter (options: circle), shutting down node...");
			n_.shutdown();
			return;
		}
	}

	void topicCallback_ft_compensated(const geometry_msgs::WrenchStampedPtr &msg)
	{
		m_ft_compensated = *msg;
	}

	void topicCallback_joint_states(const control_msgs::JointTrajectoryControllerStatePtr &msg)
	{

	}

	bool srvCallback_Start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
	{
		m_run_controller = true;
		m_t_start = ros::Time::now();

		return true;
	}

	bool srvCallback_Stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
	{
		m_run_controller = false;
		return true;
	}

private:

	CartTrajGenerator *m_cart_traj_generator;
	SurfaceTracingController *m_surface_tracing_controller;

	geometry_msgs::WrenchStamped m_ft_compensated;


	// starting time for the trajectory generator
	ros::Time m_t_start;

	bool m_run_controller;



};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dumbo_contact_point_estimation");

	DumboSurfaceTracingControllerNode dumbo_surface_tracing_controller_node;


	ros::spin();

	return 0;
}
