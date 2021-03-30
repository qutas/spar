#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <spar_msgs/FlightMotionAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>

namespace SparNode {

class Spar
{
	public:
		Spar();
		~Spar();
		void cb_act_goal();
		void cb_act_preempt();
		void cb_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void cb_timer(const ros::TimerEvent& event);

	private:
		uint16_t cancel_goal();
		uint16_t type_mask_from_motion(const uint8_t motion);
		double yaw_from_quaternion(const geometry_msgs::Quaternion q);
		double shortest_angle( const double a1, const double a2 );

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		std::string param_action_name_;
		std::string param_frame_id_;
		double param_update_rate_;
		
		ros::Timer timer_;
		ros::Publisher pub_setpoint_;
		ros::Subscriber sub_pose_;
		actionlib::SimpleActionServer<spar_msgs::FlightMotionAction> as_;
		
		bool has_pose_;
		geometry_msgs::PoseStamped pose_;
		
		mavros_msgs::PositionTarget output_;
		
		spar_msgs::FlightMotionGoal goal_;
		spar_msgs::FlightMotionFeedback feedback_;
		spar_msgs::FlightMotionResult result_;

};

}
