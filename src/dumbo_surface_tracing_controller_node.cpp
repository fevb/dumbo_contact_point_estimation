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

#include <ros/ros.h>
#include <contact_point_estimation/SurfaceTracingController.h>
#include <dumbo_cart_vel_controller/DumboCartVelController.h>
#include <cart_traj_generators/CircleTrajGenerator.h>
#include <cart_traj_generators/LineTrajGenerator.h>
#include <cart_traj_generators/LinePeriodicTrajGenerator.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>
#include <boost/bind.hpp>

class DumboSurfaceTracingControllerNode : DumboCartVelController
{

public:

	ros::Publisher topicPub_twist_ft_sensor_;
	ros::Subscriber topicSub_ft_;
	ros::Subscriber topicSub_surface_normal_;

	/// declaration of service servers
	ros::ServiceServer srvServer_Start_;
	ros::ServiceServer srvServer_Stop_;

	DumboSurfaceTracingControllerNode() : DumboCartVelController()
	{
		m_run_controller = false;

		topicSub_CommandTwist_.shutdown();

		topicPub_twist_ft_sensor_ = n_.advertise<geometry_msgs::TwistStamped>("twist_ft_sensor", 1);
		topicSub_ft_ = n_.subscribe("ft", 1,
				&DumboSurfaceTracingControllerNode::topicCallback_ft, this);
		topicSub_surface_normal_ = n_.subscribe("surface_normal_estimate", 1,
						&DumboSurfaceTracingControllerNode::topicCallback_surface_normal, this);


		srvServer_Start_ = n_.advertiseService("start", &DumboSurfaceTracingControllerNode::srvCallback_Start, this);
		srvServer_Stop_ = n_.advertiseService("stop", &DumboSurfaceTracingControllerNode::srvCallback_Stop, this);

		m_surface_tracing_controller = new SurfaceTracingController();
		m_cart_traj_generator = NULL;

		// gets ROS parameters for the surface tracing controller
		// and sets the parameters in the controller object
		configureSurfaceTracingController();

		// gets ROS parameters for the trajectory generator
		// and sets the parameters in the controller object
		configureTrajectoryGenerator();



	}

	~DumboSurfaceTracingControllerNode()
	{
		delete m_surface_tracing_controller;
		delete m_cart_traj_generator;
	}

	void configureSurfaceTracingController()
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


		if (n_.hasParam("trajectory_generator/trajectory_type"))
		{
			n_.getParam("trajectory_generator/trajectory_type", m_trajectory_type);
		}

		else
		{
			ROS_ERROR("Parameter trajectory_generator/trajectory_type not set, shutting down node...");
			n_.shutdown();
			return;
		}

		if(m_trajectory_type=="circle")
		{

			CircleTrajGenerator *circle_traj_generator = new CircleTrajGenerator();
			m_cart_traj_generator = circle_traj_generator;

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

			circle_traj_generator->setRadius(radius);
			circle_traj_generator->setPeriod(period);
			circle_traj_generator->setDuration(duration);
		}

		else if(m_trajectory_type=="line")
		{
			LineTrajGenerator *line_traj_generator = new LineTrajGenerator();
			m_cart_traj_generator = line_traj_generator;

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

			// get the tangential direction in which to move in a straight line
			KDL::Vector tangential_direction;
			XmlRpc::XmlRpcValue tangentialDirectionXmlRpc;
			if (n_.hasParam("trajectory_generator/tangential_direction"))
			{
				n_.getParam("trajectory_generator/tangential_direction", tangentialDirectionXmlRpc);
			}

			else
			{
				ROS_ERROR("Parameter trajectory_generator/tangential_direction not set, shutting down node...");
				n_.shutdown();
				return;
			}

			if(tangentialDirectionXmlRpc.size()<3)
			{
				ROS_ERROR("Parameter trajectory_generator/tangential_direction wrong size (must be 3)");
			}

			for(unsigned int i = 0; i < tangentialDirectionXmlRpc.size(); i++)
			{
				tangential_direction(i) = (double)tangentialDirectionXmlRpc[i];
			}


			KDL::Vector normal_direction;
			XmlRpc::XmlRpcValue normalDirectionXmlRpc;
			if (n_.hasParam("trajectory_generator/normal_direction"))
			{
				n_.getParam("trajectory_generator/normal_direction", normalDirectionXmlRpc);
			}

			else
			{
				ROS_ERROR("Parameter trajectory_generator/normal_direction not set, shutting down node...");
				n_.shutdown();
				return;
			}

			if(normalDirectionXmlRpc.size()<3)
			{
				ROS_ERROR("Parameter trajectory_generator/normal_direction wrong size (must be 3)");
			}

			for(unsigned int i = 0; i < normalDirectionXmlRpc.size(); i++)
			{
				normal_direction(i) = (double)normalDirectionXmlRpc[i];
			}

			// get velocity of line trajectory
			double vel;
			if (n_.hasParam("trajectory_generator/vel"))
			{
				n_.getParam("trajectory_generator/vel", vel);
			}

			else
			{
				ROS_ERROR("Parameter trajectory_generator/vel not set, shutting down node...");
				n_.shutdown();
				return;
			}

			// get amplitude of sine wave to apply in the normal direction
			double sine_amplitude;
			if (n_.hasParam("trajectory_generator/sine_amplitude"))
			{
				n_.getParam("trajectory_generator/sine_amplitude", sine_amplitude);
			}

			else
			{
				ROS_ERROR("Parameter trajectory_generator/sine_amplitude not set, shutting down node...");
				n_.shutdown();
				return;
			}


			line_traj_generator->setDuration(duration);
			line_traj_generator->setTangentialDirection(tangential_direction);
			line_traj_generator->setNormalDirection(normal_direction);
			line_traj_generator->setVel(vel);
			line_traj_generator->setSineAmplitude(sine_amplitude);
		}

		else if(m_trajectory_type=="line_periodic")
		{
			LinePeriodicTrajGenerator *line_periodic_traj_generator = new LinePeriodicTrajGenerator();
			m_cart_traj_generator = line_periodic_traj_generator;

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

			// get the tangential direction in which to move in a straight line
			KDL::Vector tangential_direction;
			XmlRpc::XmlRpcValue tangentialDirectionXmlRpc;
			if (n_.hasParam("trajectory_generator/tangential_direction"))
			{
				n_.getParam("trajectory_generator/tangential_direction", tangentialDirectionXmlRpc);
			}

			else
			{
				ROS_ERROR("Parameter trajectory_generator/tangential_direction not set, shutting down node...");
				n_.shutdown();
				return;
			}

			if(tangentialDirectionXmlRpc.size()<3)
			{
				ROS_ERROR("Parameter trajectory_generator/tangential_direction wrong size (must be 3)");
			}

			for(unsigned int i = 0; i < tangentialDirectionXmlRpc.size(); i++)
			{
				tangential_direction(i) = (double)tangentialDirectionXmlRpc[i];
			}


			KDL::Vector normal_direction;
			XmlRpc::XmlRpcValue normalDirectionXmlRpc;
			if (n_.hasParam("trajectory_generator/normal_direction"))
			{
				n_.getParam("trajectory_generator/normal_direction", normalDirectionXmlRpc);
			}

			else
			{
				ROS_ERROR("Parameter trajectory_generator/normal_direction not set, shutting down node...");
				n_.shutdown();
				return;
			}

			if(normalDirectionXmlRpc.size()<3)
			{
				ROS_ERROR("Parameter trajectory_generator/normal_direction wrong size (must be 3)");
			}

			for(unsigned int i = 0; i < normalDirectionXmlRpc.size(); i++)
			{
				normal_direction(i) = (double)normalDirectionXmlRpc[i];
			}

			// get velocity of line trajectory
			double vel;
			if (n_.hasParam("trajectory_generator/vel"))
			{
				n_.getParam("trajectory_generator/vel", vel);
			}

			else
			{
				ROS_ERROR("Parameter trajectory_generator/vel not set, shutting down node...");
				n_.shutdown();
				return;
			}

			// get amplitude of sine wave to apply in the normal direction
			double sine_amplitude;
			if (n_.hasParam("trajectory_generator/sine_amplitude"))
			{
				n_.getParam("trajectory_generator/sine_amplitude", sine_amplitude);
			}

			else
			{
				ROS_ERROR("Parameter trajectory_generator/sine_amplitude not set, shutting down node...");
				n_.shutdown();
				return;
			}

			// get period of the line trajectory
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


			line_periodic_traj_generator->setDuration(duration);
			line_periodic_traj_generator->setTangentialDirection(tangential_direction);
			line_periodic_traj_generator->setNormalDirection(normal_direction);
			line_periodic_traj_generator->setVel(vel);
			line_periodic_traj_generator->setSineAmplitude(sine_amplitude);
			line_periodic_traj_generator->setPeriod(period);
		}

		else
		{
			ROS_ERROR("Invalid trajectory_type parameter (options: circle, line, line_periodic), shutting down node...");
			n_.shutdown();
			return;
		}
	}

	void topicCallback_ft(const geometry_msgs::WrenchStampedPtr &msg)
	{
		m_ft_mutex.lock();
		m_ft = *msg;
		m_ft_mutex.unlock();

		std::string ft_sensor_frame_id;
		if(m_ft.header.frame_id.substr(0,1)=="/")
		{
			ft_sensor_frame_id = m_ft.header.frame_id.erase(0, 1);
		}

		else
		{
			ft_sensor_frame_id = m_ft.header.frame_id;
		}

		if(!m_dumbo_ft_kdl_wrapper.isInitialized())
		{
			if(m_dumbo_ft_kdl_wrapper.init("arm_base_link", ft_sensor_frame_id))
			{
				m_dumbo_ft_kdl_wrapper.ik_solver_vel->setLambda(0.3);
			}
		}
	}

	void topicCallback_surface_normal(const geometry_msgs::Vector3StampedPtr &msg)
	{
		m_surface_normal_mutex.lock();
		m_surface_normal = *msg;
		m_surface_normal_mutex.unlock();
	}

	void publishJointVelThreadFunc()
	{
		static ros::Rate loop_rate(m_surface_tracing_controller->getControlFrequency());
		for(;;)
		{
			if(m_run_controller)
			{
				KDL::JntArray q_dot;
				if(calculateJointVelCommand(q_dot))
				{

					// publish joint velocity command to the manipulator
					publishJointVelCommand(q_dot);
					// publish twist of the FT sensor frame
					publishFTSensorTwist();
				}
				loop_rate.sleep();
			}

			else
			{
				return;
			}
		}

	}

	// calculate twist of FT sensor, then use IK to calculate joint velocities
	bool calculateJointVelCommand(KDL::JntArray &q_dot)
	{
		// check that KDL wrappers are initialized
		if(!m_dumbo_ft_kdl_wrapper.isInitialized())
		{
			static ros::Time t = ros::Time::now();
			if((ros::Time::now()-t).toSec()>2.0)
			{
				ROS_ERROR("Dumbo KDL wrapper (arm_base_link to ft_sensor) not initialized");
				t = ros::Time::now();
			}
			return false;
		}

		// collect joint pos and vel
		KDL::JntArrayVel q_in(m_DOF);
		m_joint_state_mutex.lock();
		for(unsigned int i=0; i<m_DOF; i++)
		{
			q_in.q(i) = m_joint_state_msg.actual.positions[i];
			q_in.qdot(i) = m_joint_state_msg.actual.velocities[i];
		}
		m_joint_state_mutex.unlock();

		// calculate pose and twist of FT sensor

		KDL::FrameVel Fvel_ft;
		m_dumbo_ft_kdl_wrapper.fk_solver_vel->JntToCart(q_in, Fvel_ft);

		Eigen::Vector3d p(Fvel_ft.p.p(0), Fvel_ft.p.p(1), Fvel_ft.p.p(2));
		m_twist_mutex.lock();
		m_twist_ft_sensor.vel = Fvel_ft.p.v;
		m_twist_ft_sensor.rot = Fvel_ft.M.w;
		m_twist_mutex.unlock();

		// get p_d, p_dot_d set points from the trajectory generator
		double time = (ros::Time::now()-m_t_start).toSec();
		KDL::Frame F;
		KDL::Twist v;
		m_cart_traj_generator->getSetPoint(time, F, v);

		Eigen::Vector3d p_d;
		Eigen::Vector3d p_dot_d;

		for(unsigned int i=0; i<3; i++) p_d(i) = F.p(i);
		for(unsigned int i=0; i<3; i++) p_dot_d(i) = v.vel(i);


		// calculate control signal (twist of FT sensor with respect to arm_base_frame)

		// convert F/T measurements to the base frame
		KDL::Wrench ft;
		m_ft_mutex.lock();
		tf::wrenchMsgToKDL(m_ft.wrench, ft);
		m_ft_mutex.unlock();

		KDL::Wrench ft_base = Fvel_ft.M.R*ft;


		Eigen::Matrix<double, 6, 1> ft_base_;
		tf::wrenchKDLToEigen(ft_base, ft_base_);


		Eigen::Vector3d surface_normal;
		m_surface_normal_mutex.lock();
		tf::vectorMsgToEigen(m_surface_normal.vector, surface_normal);
		m_surface_normal_mutex.unlock();


		Eigen::Vector3d u;
		u = m_surface_tracing_controller->controlSignal(surface_normal,
				ft_base_, p, p_d, p_dot_d);

		// calculate inverse kinematics
		// set rotational vel to zero
		KDL::Twist u_ = KDL::Twist::Zero();
		for(unsigned int i=0; i<3; i++) u_(i) = u(i);

		m_dumbo_ft_kdl_wrapper.ik_solver_vel->CartToJnt(q_in.q, u_, q_dot);

		return true;
	}

	void publishFTSensorTwist()
	{
		geometry_msgs::TwistStamped twist_ft_sensor;
		twist_ft_sensor.header.frame_id = "arm_base_link";
		twist_ft_sensor.header.stamp = ros::Time::now();
		m_twist_mutex.lock();
		tf::twistKDLToMsg(m_twist_ft_sensor, twist_ft_sensor.twist);
		m_twist_mutex.unlock();

		topicPub_twist_ft_sensor_.publish(twist_ft_sensor);
	}

	bool srvCallback_Start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
	{
		if(!m_dumbo_ft_kdl_wrapper.isInitialized())
		{
			ROS_ERROR("Cannot start CPE, KDL wrapper not initialized");
			return false;
		}

		if(!m_received_js)
		{
			static ros::Time t = ros::Time::now();
			if((ros::Time::now()-t).toSec()>2.0)
			{
				ROS_ERROR("Haven't received joint states, can't start controller...");
				t = ros::Time::now();
			}
			return false;
		}

		else
		{
			ROS_INFO("Starting surface tracing controller");

			m_run_controller = true;
			m_t_start = ros::Time::now();
			m_surface_tracing_controller->reset();
			// set initial pose of FT sensor for trajectory generator
			static KDL::JntArray q_in(m_DOF);
			m_joint_state_mutex.lock();
			for(unsigned int i=0; i<m_DOF; i++) q_in(i) = m_joint_state_msg.actual.positions[i];
			m_joint_state_mutex.unlock();
			KDL::Frame F_ft_sensor;
			m_dumbo_ft_kdl_wrapper.fk_solver_pos->JntToCart(q_in, F_ft_sensor);

			configureSurfaceTracingController();
			configureTrajectoryGenerator();

			m_cart_traj_generator->setInitPose(F_ft_sensor);

			m_joint_vel_command_publish_thread = boost::thread(boost::bind(&DumboSurfaceTracingControllerNode::publishJointVelThreadFunc,
					this));
		}

		return true;
	}

	bool srvCallback_Stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
	{
		ROS_INFO("Stopping surface tracing controller");
		m_run_controller = false;
		return true;
	}

private:

	CartTrajGenerator *m_cart_traj_generator;
	SurfaceTracingController *m_surface_tracing_controller;

	// for calculating kinematics from base frame to the FT sensor frame
	KDLWrapper m_dumbo_ft_kdl_wrapper;

	geometry_msgs::WrenchStamped m_ft;
	geometry_msgs::Vector3Stamped m_surface_normal;

	// twist of the FT sensor expressed in the base frame
	KDL::Twist m_twist_ft_sensor;

	// specifies type of trajectory (circle, ...)
	std::string m_trajectory_type;

	// mutexes for protecting shared mem variables
	boost::mutex m_ft_mutex;
	boost::mutex m_surface_normal_mutex;
	boost::mutex m_twist_mutex;

	// thread that publishes joint velocity commands
	boost::thread m_joint_vel_command_publish_thread;


	// starting time for the trajectory generator
	ros::Time m_t_start;

	bool m_run_controller;


};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dumbo_contact_point_estimation");

	DumboSurfaceTracingControllerNode dumbo_surface_tracing_controller_node;

	ros::AsyncSpinner s(3);
	s.start();

	ros::waitForShutdown();

	return 0;
}
