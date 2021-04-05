#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <spar_msgs/FlightMotionAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>

namespace SparNode {

//Common defines (just to let the compiler work out how to handle them
#define TYPE_MASK_POS_MODE ( mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW_RATE )
#define TYPE_MASK_VEL_MODE ( mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW_RATE )
#define FLIGHT_MODE_OFFBOARD "OFFBOARD"

class Spar
{
	public:
		Spar();
		~Spar();

	private:
		void cb_act_goal();
		void cb_act_preempt();
		void cb_mav_state(const mavros_msgs::State::ConstPtr& msg);
		void cb_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void cb_timer(const ros::TimerEvent& event);

		void check_start_as();				//Check to see if we should start up the action server
		void stop_at_current_location();	//Set the output to be stopped at the current location
		void cancel_goal();					//Cancel the current goal (if active) and stop at current location, also called if we're changing state
		uint16_t type_mask_from_motion(const uint8_t motion);			//Decodes our motion to set an appropriate type mask
		bool is_waiting_for_convergence();	//Will return true if we are close enough to the waypoint limits to count as "reached"

		//Math Helpers
		//Sane calculations to get travel time
		ros::Duration duration_from_distance(const double dist, const double vel);
		ros::Duration duration_from_distance_2d(const double dist_a, const double dist_b, const double vel);
		//Angle calculations
		double yaw_from_quaternion(const geometry_msgs::Quaternion& q);	//Get a yaw value from a quaternion
		double shortest_angle( const double a1, const double a2 );		//Get a "shortest distance" for the yaw greater-circle problem
		double wrap_pi(const double a);									//Returns an angle between -pi and pi
		double calc_unwraped_yaw(const double start, const double end);	//Used to calculate the shortest linear increase in angles (i.e. -3.0 to 3.0 is non linear if wrapping around -pi), which means we can perform linear interpolation on our delta-yaw, and have nice smooth motion

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		std::string param_action_name_;
		std::string param_frame_id_;
		double param_update_rate_;

		ros::Timer timer_;
		ros::Publisher pub_setpoint_;
		ros::Subscriber sub_pose_;
		ros::Subscriber sub_mav_state_;
		actionlib::SimpleActionServer<spar_msgs::FlightMotionAction> as_;

		ros::Time stamp_start_;
		geometry_msgs::Point start_pos_;
		double start_yaw_;
		ros::Duration motion_duration_;
		geometry_msgs::Point end_pos_;
		double end_yaw_;

		bool takeoff_complete_;
		bool landing_complete_;

		bool is_running_;
		geometry_msgs::PoseStamped pose_;
		mavros_msgs::State mav_state_;

		mavros_msgs::PositionTarget output_;

		spar_msgs::FlightMotionGoal goal_;
		spar_msgs::FlightMotionFeedback feedback_;
		spar_msgs::FlightMotionResult result_;
};

}
