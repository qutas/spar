#include <ros/ros.h>
#include <math.h>

#include <spar_node/Spar.h>

#include <spar_msgs/FlightMotionAction.h>
#include <actionlib/server/simple_action_server.h>
#include <mavros_msgs/PositionTarget.h>

namespace SparNode {

Spar::Spar() :
	nhp_("~"),
	param_action_name_("flight"),
	param_frame_id_("map"),
	param_update_rate_(20.0),
	has_pose_(false),
	as_(nhp_, param_action_name_, false) {
    
	nhp_.param("update_rate", param_update_rate_, param_update_rate_);
	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	
	output_.header.frame_id = param_frame_id_;
	// Start up in local NED, position mode
	output_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	output_.type_mask = type_mask_from_motion(spar_msgs::FlightMotionGoal::MOTION_STOP);
	
	//register the goal and feeback callbacks
	as_.registerGoalCallback(boost::bind(&Spar::cb_act_goal, this));
	as_.registerPreemptCallback(boost::bind(&Spar::cb_act_preempt, this));

	sub_pose_ = nhp_.subscribe<geometry_msgs::PoseStamped>("pose", 100, &Spar::cb_pose, this);
	pub_setpoint_ = nhp_.advertise<mavros_msgs::PositionTarget>("setpoint_raw/local", 100);
	timer_ = nhp_.createTimer(ros::Duration(1.0/param_update_rate_), &Spar::cb_timer, this);
	
	ROS_INFO("Started spar node!");
}

Spar::~Spar(void) {
}

uint16_t Spar::type_mask_from_motion(const uint8_t motion) {
	uint16_t type_mask = 0;
	
	switch(motion) {
		//All GOTO and STOP functions work in position-hold mode
		case spar_msgs::FlightMotionGoal::MOTION_GOTO:
		case spar_msgs::FlightMotionGoal::MOTION_GOTO_POS:
		case spar_msgs::FlightMotionGoal::MOTION_GOTO_YAW:
		case spar_msgs::FlightMotionGoal::MOTION_STOP: {
			type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
						mavros_msgs::PositionTarget::IGNORE_VY |
						mavros_msgs::PositionTarget::IGNORE_VZ |
						mavros_msgs::PositionTarget::IGNORE_AFX |
						mavros_msgs::PositionTarget::IGNORE_AFY |
						mavros_msgs::PositionTarget::IGNORE_AFZ |
						mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
		
			break;
		}
		//TAKEOFF and LAND functions work in velocity-hold mode
		case spar_msgs::FlightMotionGoal::MOTION_TAKEOFF: {
		case spar_msgs::FlightMotionGoal::MOTION_LAND: {
			type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
						mavros_msgs::PositionTarget::IGNORE_PY |
						mavros_msgs::PositionTarget::IGNORE_PZ |
						mavros_msgs::PositionTarget::IGNORE_AFX |
						mavros_msgs::PositionTarget::IGNORE_AFY |
						mavros_msgs::PositionTarget::IGNORE_AFZ |
						mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

			break;
		}
		default : {
			//Undefined behaviour
			//XXX: This is an internal check, don't rely on it for user interraction
			ROS_ASSERT_MSG(false, sprintf("Undefined motion requested: %i", motion));
		}
	}
	
	return type_mask;
}

void Spar::cancel_goal() {
	//XXX: Tell the system to stop in place
	output_.type_mask = type_mask_from_motion(par_msgs::FlightMotionGoal::MOTION_STOP);
	output_.position = pose_.pose_.position;
	output_.yaw = yaw_from_quaternion(pose_.pose_.orientation);
	
	result_.final_position = output_.position;
	result_.final_yaw = output_.yaw;
	
	if( as_.isActive() )
		as_.setAborted(result_);
		
	ROS_WARN("%s action: Cancelled", param_action_name_.c_str());
}

double Spar::yaw_from_quaternion(const geometry_msgs::Quaternion q) {
	return std::atan2(2.0 * (q.z * q.w + q.x * q.y) , - 1.0 + 2.0 * (q.w * q.w + q.x * q.x));
}

double Spar::shortest_angle( const double a1, const double a2 )
{
    double diff = ( a2 - a1 + 180 ) % 360 - 180;
    return diff < -180 ? diff + 360 : diff;
}

void Spar::cb_act_goal() {
	// accept the new goal
	goal_ = *(as_.acceptNewGoal());
	
	if( (goal_.motion != spar_msgs::FlightMotionGoal::MOTION_STOP) ||
		(goal_.motion != spar_msgs::FlightMotionGoal::MOTION_GOTO) ||
		(goal_.motion != spar_msgs::FlightMotionGoal::MOTION_GOTO_POS) ||
		(goal_.motion != spar_msgs::FlightMotionGoal::MOTION_GOTO_YAW) ||
		(goal_.motion != spar_msgs::FlightMotionGoal::MOTION_TAKEOFF) ||
		(goal_.motion != spar_msgs::FlightMotionGoal::MOTION_LAND) ) {

		cancel_goal();
		ROS_ERROR("Undefined motion requested: %i", motion);
	} else {
		//Set up our type mask to perform the requested motion
		output_.type_mask = type_mask_from_motion(goal_.motion);
	}
	
	if( (goal_.velocity_vertical < 0.0) || (goal_.velocity_horizontal < 0.0) || (goal_.yawrate < 0.0)) {
		cancel_goal();
		ROS_ERROR("Velocity information not valid (%0.2f, %0.2f, %0.2f)", goal_.velocity_vertical, goal_.velocity_horizontal, goal_.yawrate);
	} else {
		//TODO: Take timestamps of start & end times, set up for "setpoint mode" if needed
	}
	
	if( (goal_. wait_for_convergence) && ( (goal_.position_radius <= 0.0) || (goal_.yaw_range <= 0.0) ) ) {
		cancel_goal();
		ROS_ERROR("Radius/range information not valid (%0.2f, %0.2f)", goal_.position_radius, goal_.yaw_range);
	} // XXX: All values calculated later in the callback
	
	//If after all of that, we're still good to go, let the user know!
	if( as_.isActive() )
		ROS_INFO("%s action: Accepted", param_action_name_.c_str());
}

void Spar::cb_act_preempt() {
	if(as_.isActive()) {
		cancel_goal();
	} else {
		ROS_INFO("%s action: Preempted", param_action_name_.c_str());
		// set the action state to preempted
		as_.setPreempted();
	}
}

void Spar::cb_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if(!has_pose_) {
		ROS_INFO("Recieved pose, starting action server!");
		has_pose_ = true;
		as_.start();
	}
	
	pose_ = *msg;
}

void Spar::cb_timer(const ros::TimerEvent& event) {
    // If we are running an action, then load in from our action
	// otherwise we're holding location, so don't touch output
    if ( as_.isActive() ) {
		//TODO: calculate the current position of our lerp line
		//TODO: fill in goal positions
		double a_v = 0.0;	// Alpha Vertical
		double a_h = 0.0;	// Alpha Horizontal
		double a_y = 0.0;	// Alpha Yaw
		
		//Prepare our feedback
		// (but don't send yet as we want to write 
		//  waiting_for_convergence from "alpha > 1.0" test)
		feedback_.goal_position = output_.position;
		feedback_.goal_yaw = output_.yaw;
		feedback_.progress = alpha;
		feedback_.waiting_for_convergence = false;
	
		//Check our success cases (goal tracking has reached end-points)
		//XXX: This will happen imidiately if velocities are 0
		if ( (a_v >= 1.0) && (a_h >= 1.0) && (a_y >= 1.0) ) {
			bool success = false;
			
			//Check to see if we should wait
			if (goal_.wait_for_convergence) {
				double dpx = goal_.position.x - pose_.pose.position.x;
				double dpy = goal_.position.y - pose_.pose.position.y;
				double dpz = goal_.position.z - pose_.pose.position.z;
				double dp = std::sqrt(dpx*dpx + dpy*dpy + dpz*dpz);
				double dy = shortest_angle(goal_.yaw, yaw_from_quaternion(pose_.pose_.orientation));
							
				if( (dp < goal_.position_radius) && (dy < goal_.yaw_range) ) {
					success = true;
				} else {
					feedback_.waiting_for_convergence = true;
				}
			} else {
				//Else we've reached then end and that's all!
				success = true;
			}
			
			if(success) {
				ROS_INFO("%s action: Succeeded", param_action_name_.c_str());
				
				//XXX:	MOTION_GOTO, MOTION_GOTO_POS, MOTION_GOTO_YAW, MOTION_STOP:
				//		No changes necessary, stick with last postiion
				//XXX:	MOTION_TAKEOFF, MOTION_LAND
				//		Switch to "hold current location"
				if( (goal_.motion == spar_msgs::FlightMotionGoal::MOTION_TAKEOFF) || 
					(goal_.motion == spar_msgs::FlightMotionGoal::MOTION_LAND) ) {
		
					output_.type_mask = type_mask_from_motion(spar_msgs::FlightMotionGoal::MOTION_STOP);
					output_.position = pose_.pose.position;
					output_.yaw = yaw_from_quaternion(pose_.pose_.orientation);
				}
				
				// Fill in details and set the action state to succeeded
				result_.final_position = output_.position;
				result_.final_yaw = output_.yaw;
				as_.setSucceeded(result_);
			}
		
		}
	
		//Output our feedback
		// XXX: Doesn't matter about sending it late,
		//      ROS does not make those guarentees anyway
		as_.publishFeedback(feedback_);
	}
	
	/*
	//TODO: else if (not armed) {
		ROS_WARN("Disarm detected!");
		cancel_goal();
	*/
	
	output_.header.stamp = ros::Time::now();
	pub_setpoint_.publish(output_);
}

}
