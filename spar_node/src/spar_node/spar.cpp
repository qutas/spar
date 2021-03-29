#include <ros/ros.h>

#include <spar_node/Spar.h>

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
	output_.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
						mavros_msgs::PositionTarget::IGNORE_VY |
						mavros_msgs::PositionTarget::IGNORE_VZ |
						mavros_msgs::PositionTarget::IGNORE_AFX |
						mavros_msgs::PositionTarget::IGNORE_AFY |
						mavros_msgs::PositionTarget::IGNORE_AFZ |
						mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
	
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

void Spar::cb_act_goal() {
	// accept the new goal
	goal_ = *(as_.acceptNewGoal());
}

void Spar::cb_act_preempt() {
	ROS_INFO("%s: Preempted", param_action_name_.c_str());
	// set the action state to preempted
	as_.setPreempted();
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
    if (as_.isActive()) {		
		//TODO: calculate the current position of our lerp line
		//TODO: fill in goal positions
		double alpha = 0.0;
		
		//Prepare our feedback
		// (but don't send yet as we want to write 
		//  waiting_for_convergence from "alpha > 1.0" test)
		feedback_.goal_position = output_.position;
		feedback_.goal_yaw = output_.yaw;
		feedback_.progress = alpha;
		feedback_.waiting_for_convergence = false;
	
		if (alpha >= 1.0) {
			bool success = false;
			
			//Check to see if we should wait
			if (goal_.wait_for_convergence) {
				double dp = 0.0; //math::sqrt(...);
				double dy = 0.0; //shortest_path(...);
			
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
				ROS_INFO("%s: Succeeded", param_action_name_.c_str());
				
				//TODO: Reset output to hold at final/current location
				
				// Fill in details and set the action state to succeeded
				result_.final_position = output_.position;
				result_.final_yaw = output_.yaw;
				as_.setSucceeded(result_);
			}
		
		} /* //TODO: else if (not armed) {
			
			//Abort (failsafe / disarm)
			ROS_INFO("%s: Aborted", action_name_.c_str());
			result_.final_position = output_.position;
			result_.final_yaw = output_.yaw;
			as_.setAborted(result_);
		} */
	
		//Output our feedback
		// XXX: Doesn't matter about sending it late,
		//      ROS does not make those guarentees anyway
		as_.publishFeedback(feedback_);
	}
	
	output_.header.stamp = ros::Time::now();
	pub_setpoint_.publish(output_);
}

}
