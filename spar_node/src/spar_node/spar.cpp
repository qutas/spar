#include <ros/ros.h>
#include <math.h>

#include <spar_node/Spar.h>

#include <spar_msgs/FlightMotionAction.h>
#include <actionlib/server/simple_action_server.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Point.h>

namespace SparNode {

Spar::Spar() :
	nhp_("~"),
	param_action_name_("flight"),
	param_frame_id_("map"),
	param_update_rate_(20.0),
	is_running_(false),
	takeoff_complete_(false),
	landing_complete_(false),
	stamp_start_(0),
	start_yaw_(0.0),
	motion_duration_(0),
	end_yaw_(0.0),
	as_(nhp_, param_action_name_, false) {

	//Sanitise our structures
	start_pos_.x = 0.0;
	start_pos_.y = 0.0;
	start_pos_.z = 0.0;
	end_pos_.x = 0.0;
	end_pos_.y = 0.0;
	end_pos_.z = 0.0;

	//Load in our variables
	nhp_.param("update_rate", param_update_rate_, param_update_rate_);
	nhp_.param("frame_id", param_frame_id_, param_frame_id_);

	output_.header.frame_id = param_frame_id_;
	// Start up in local NED, position mode
	output_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	output_.type_mask = type_mask_from_motion(spar_msgs::FlightMotionGoal::MOTION_STOP);

	pose_.pose.orientation.w = 1.0; //XXX: Give a valid quaternion to start with

	//register the goal and feeback callbacks
	as_.registerGoalCallback(boost::bind(&Spar::cb_act_goal, this));
	as_.registerPreemptCallback(boost::bind(&Spar::cb_act_preempt, this));

	sub_pose_ = nhp_.subscribe<geometry_msgs::PoseStamped>("pose", 100, &Spar::cb_pose, this);
	sub_mav_state_ = nhp_.subscribe<mavros_msgs::State>("mav_state", 100, &Spar::cb_mav_state, this);
	pub_setpoint_ = nhp_.advertise<mavros_msgs::PositionTarget>("setpoint", 100);
	timer_ = nhp_.createTimer(ros::Duration(1.0/param_update_rate_), &Spar::cb_timer, this);

	ROS_INFO("[Spar] Started spar node!");
}

Spar::~Spar(void) {
	/*
	if(as_.isActive())
		cancel_goal();
	*/
}

uint16_t Spar::type_mask_from_motion(const uint8_t motion) {
	uint16_t type_mask = 0;

	switch(motion) {
		//All GOTO and STOP functions work in position-hold mode
		case spar_msgs::FlightMotionGoal::MOTION_GOTO:
		case spar_msgs::FlightMotionGoal::MOTION_GOTO_POS:
		case spar_msgs::FlightMotionGoal::MOTION_GOTO_YAW:
		case spar_msgs::FlightMotionGoal::MOTION_STOP: {
			type_mask = TYPE_MASK_POS_MODE;

			break;
		}
		//TAKEOFF and LAND functions work in velocity-hold mode
		case spar_msgs::FlightMotionGoal::MOTION_TAKEOFF:
		case spar_msgs::FlightMotionGoal::MOTION_LAND: {
			type_mask = TYPE_MASK_VEL_MODE;

			break;
		}
		default : {
			//Undefined behaviour
			//XXX: This is an internal check, don't rely on it for user interraction
			ROS_ASSERT_MSG(false, "[Spar] Undefined motion requested: %i", goal_.motion);
		}
	}

	return type_mask;
}

void Spar::stop_at_current_location() {
	//XXX: Tell the system to stop in place
	output_.type_mask = type_mask_from_motion(spar_msgs::FlightMotionGoal::MOTION_STOP);
	output_.position = pose_.pose.position;
	output_.yaw = yaw_from_quaternion(pose_.pose.orientation);
}

void Spar::cancel_goal() {
	stop_at_current_location();

	if( as_.isActive() ) {
		result_.final_position = output_.position;
		result_.final_yaw = output_.yaw;
		as_.setAborted(result_);

		ROS_WARN("[Spar] %s action: Cancelled", param_action_name_.c_str());
	}
}

double Spar::yaw_from_quaternion(const geometry_msgs::Quaternion& q) {
	return std::atan2(2.0 * (q.z * q.w + q.x * q.y) , - 1.0 + 2.0 * (q.w * q.w + q.x * q.x));
}

double Spar::shortest_angle( const double a1, const double a2 ) {
    double diff = std::fmod( ( a2 - a1 + M_PI ), 2*M_PI) - M_PI;
    return diff < -M_PI ? diff + 2*M_PI : diff;
}

double Spar::wrap_pi(const double a) {
     // already in range
     if (-M_PI <= a && a <= M_PI) {
         return a;
     }

     const double range = 2*M_PI;
     const double inv_range = 1.0 / range;
     const double num_wraps = std::floor((a + M_PI) * inv_range);
     return a - range * num_wraps;
}

double Spar::calc_unwraped_yaw(const double start, const double end) {
	double diff = end - start;
	return (std::fabs(diff) < M_PI) ? start + diff :
		   (start > end) ? start + fabs(shortest_angle(end, start)) :
						   start - fabs(shortest_angle(end, start));
}

ros::Duration Spar::duration_from_distance(const double dist, const double vel) {
	return ros::Duration( std::fabs(dist / vel) );
}

ros::Duration Spar::duration_from_distance_2d(const double dist_a, const double dist_b, const double vel) {
	return duration_from_distance(std::sqrt(dist_a*dist_a + dist_b*dist_b), vel);
}

void Spar::cb_act_goal() {
	// accept the new goal
	goal_ = *(as_.acceptNewGoal());

	if(!is_running_) {
		//We're not yet ready to accept goals
		//XXX:	This should be taken care of by "as_.shutdown()",
		//		but that doesn't seem to work
		cancel_goal();
		ROS_ERROR("[Spar] Rejecting goal: aircraft not ready for offboard control (%s, %s, %s)",
				  mav_state_.connected ? "Connected" : "Disonnected",
				  mav_state_.mode.c_str(),
				  mav_state_.armed ? "Armed" : "Disarmed");
	}

	if( (goal_.motion != spar_msgs::FlightMotionGoal::MOTION_STOP) &&
		(goal_.motion != spar_msgs::FlightMotionGoal::MOTION_GOTO) &&
		(goal_.motion != spar_msgs::FlightMotionGoal::MOTION_GOTO_POS) &&
		(goal_.motion != spar_msgs::FlightMotionGoal::MOTION_GOTO_YAW) &&
		(goal_.motion != spar_msgs::FlightMotionGoal::MOTION_TAKEOFF) &&
		(goal_.motion != spar_msgs::FlightMotionGoal::MOTION_LAND) ) {

		cancel_goal();
		//Something is broken? Forgot to update this with a new mode addition?
		ROS_ERROR("[Spar] Rejecting goal: Undefined motion requested: %i", goal_.motion);
	} else {
		//Set up our type mask to perform the requested motion
		output_.type_mask = type_mask_from_motion(goal_.motion);
	}

	if( (goal_.velocity_vertical < 0.0) || (goal_.velocity_horizontal < 0.0) || (goal_.yawrate < 0.0)) {
		cancel_goal();
		ROS_ERROR("[Spar] Rejecting goal: Velocity information not valid (%0.2f, %0.2f, %0.2f)", goal_.velocity_vertical, goal_.velocity_horizontal, goal_.yawrate);
	} else if ( ( (goal_.motion != spar_msgs::FlightMotionGoal::MOTION_TAKEOFF) ||
				  (goal_.motion != spar_msgs::FlightMotionGoal::MOTION_LAND) ) &&
				( (goal_.velocity_vertical == 0.0) ) ) {
		cancel_goal();
		ROS_ERROR("[Spar] Rejecting goal: Vertical velocity information require for takeoff/landing (%0.2f)", goal_.velocity_vertical);
	}

	if( (goal_. wait_for_convergence) && ( (goal_.position_radius <= 0.0) || (goal_.yaw_range <= 0.0) ) ) {
		cancel_goal();
		ROS_ERROR("[Spar] Rejecting goal: Radius/range information not valid (%0.2f, %0.2f)", goal_.position_radius, goal_.yaw_range);
	} // XXX: All values calculated later in the callback

	// If after all of that, we're still good to go
	// Perform the rest of our setup calculations
	//XXX: This should set stamp_start_, start_pos_, start_yaw_, motion_duration_, end_pos_, end_yaw_
	if( as_.isActive() ) {
		stamp_start_ = ros::Time::now();
		start_pos_ = pose_.pose.position;
		start_yaw_ = yaw_from_quaternion(pose_.pose.orientation);

		//Define our start and ends for each mode of motion
		//XXX: Each should set motion_duration_, end_pos_, end_yaw_
		if (goal_.motion == spar_msgs::FlightMotionGoal::MOTION_STOP) {
			motion_duration_ = ros::Duration(0);
			end_pos_ = start_pos_;
			end_yaw_ = start_yaw_;
		} else if (goal_.motion == spar_msgs::FlightMotionGoal::MOTION_GOTO) {
			end_pos_ = goal_.position;
			end_yaw_ = calc_unwraped_yaw(start_yaw_, goal_.yaw);

			double dx = (end_pos_.x - start_pos_.x);
			double dy = (end_pos_.y - start_pos_.y);
			double dz = (end_pos_.z - start_pos_.z);
			double dyaw = shortest_angle(end_yaw_, start_yaw_);

			ros::Duration dt_xy = (goal_.velocity_horizontal == 0.0) ? ros::Duration(0) : duration_from_distance_2d(dx, dy, goal_.velocity_horizontal);
			ros::Duration dt_z =  (goal_.velocity_vertical == 0.0) ? ros::Duration(0) : duration_from_distance(dz, goal_.velocity_vertical);
			ros::Duration dt_yaw = (goal_.yawrate == 0.0) ? ros::Duration(0) : duration_from_distance(dyaw, goal_.yawrate);

			motion_duration_ = ( (dt_xy > dt_z) && (dt_xy > dt_yaw) ) ? dt_xy : ( dt_z > dt_yaw ) ? dt_z : dt_yaw;
        	// ROS_INFO("[Spar] dur: [%0.2f, %0.2f, %0.2f; %0.2f]", dt_xy.toSec(), dt_z.toSec(), dt_yaw.toSec(), motion_duration_.toSec(1);
		} else if (goal_.motion == spar_msgs::FlightMotionGoal::MOTION_GOTO_POS) {
			end_pos_ = goal_.position;
			end_yaw_ = start_yaw_;

			double dx = (end_pos_.x - start_pos_.x);
			double dy = (end_pos_.y - start_pos_.y);
			double dz = (end_pos_.z - start_pos_.z);

			ros::Duration dt_xy = (goal_.velocity_horizontal == 0.0) ? ros::Duration(0) : duration_from_distance_2d(dx, dy, goal_.velocity_horizontal);
			ros::Duration dt_z =  (goal_.velocity_vertical == 0.0) ? ros::Duration(0) : duration_from_distance(dz, goal_.velocity_vertical);

			motion_duration_ = (dt_xy > dt_z) ? dt_xy : dt_z;
		} else if (goal_.motion == spar_msgs::FlightMotionGoal::MOTION_GOTO_YAW) {
			end_pos_ = start_pos_;
			end_yaw_ = calc_unwraped_yaw(start_yaw_, goal_.yaw);

			double dyaw = shortest_angle(end_yaw_, start_yaw_);

			motion_duration_ = (goal_.yawrate == 0.0) ? ros::Duration(0) : duration_from_distance(dyaw, goal_.yawrate);
		} else if (goal_.motion == spar_msgs::FlightMotionGoal::MOTION_TAKEOFF) {
			//Fly straight up
			end_pos_.x = start_pos_.x;
			end_pos_.y = start_pos_.y;
			end_pos_.z = goal_.position.z;
			end_yaw_ = start_yaw_;

			double dz = (end_pos_.z - start_pos_.z);
			//XXX: Motion duration gives "expected" take-off time, then switch to position hold
			motion_duration_ =  duration_from_distance(dz, goal_.velocity_vertical);	//XXX: velocity_vertical already checked to be >0
		} else if (goal_.motion == spar_msgs::FlightMotionGoal::MOTION_LAND) {
			//Fly straight down
			end_pos_.x = start_pos_.x;
			end_pos_.y = start_pos_.y;
			end_pos_.z = 0.0;	//XXX: We actually just keep descending until disarm, but this is the 'goal'
			end_yaw_ = start_yaw_;

			double dz = (end_pos_.z - start_pos_.z);
			//XXX: Motion duration gives "expected" take-off time, but this is only used for feedback estimate
			motion_duration_ =  duration_from_distance(dz, goal_.velocity_vertical);	//XXX: velocity_vertical already checked to be >0
		} else {
			//Soemthing is broken, forgot to update this with a new mode addition?
			ROS_ASSERT_MSG(false, "[Spar] Undefined motion set: %i", goal_.motion);
		}

		ROS_INFO("[Spar] Selected duration: %0.2f", motion_duration_.toSec());
		ROS_INFO("[Spar] Selected motion: [%0.2f, %0.2f, %0.2f; %0.2f] -> [%0.2f, %0.2f, %0.2f; %0.2f]", start_pos_.x, start_pos_.y, start_pos_.z, start_yaw_, end_pos_.x, end_pos_.y, end_pos_.z, end_yaw_);

		ROS_INFO("[Spar] %s action: Accepted", param_action_name_.c_str());
	}
}

void Spar::cb_act_preempt() {
	if(as_.isActive()) {
		cancel_goal();
	} else {
		ROS_INFO("[Spar] %s action: Preempted", param_action_name_.c_str());
		// set the action state to preempted
		as_.setPreempted();
	}
}

void Spar::check_start_as() {
	//Do nothing if already running
	if(is_running_)
		return;

	//If we've had a valid input...
	// and we're connected...
	// and our flight mode looks good...
	// and we're armed...
	if( (pose_.header.stamp != ros::Time(0)) &&
		(mav_state_.header.stamp != ros::Time(0)) &&
		(mav_state_.connected) &&
		(mav_state_.mode == FLIGHT_MODE_OFFBOARD) &&
		(mav_state_.armed) ) {

		//Then we're good to go!
		is_running_ = true;
		as_.start();
		ROS_INFO("[Spar] Started action server!");
	} //else, not yet
}

void Spar::cb_mav_state(const mavros_msgs::State::ConstPtr& msg) {
	if(mav_state_.header.stamp == ros::Time(0))
		ROS_INFO("[Spar] Recieved mav_state...");

	mav_state_ = *msg;
}

void Spar::cb_pose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	if(pose_.header.stamp == ros::Time(0))
		ROS_INFO("[Spar] Recieved pose...");

	pose_ = *msg;
}

bool Spar::is_waiting_for_convergence() {
	bool is_waiting = false;

	//Check to see if we should wait
	if (goal_.wait_for_convergence) {
		double dpx = end_pos_.x - pose_.pose.position.x;
		double dpy = end_pos_.y - pose_.pose.position.y;
		double dpz = end_pos_.z - pose_.pose.position.z;
		double dp = std::sqrt(dpx*dpx + dpy*dpy + dpz*dpz);
		double dy = shortest_angle(end_yaw_, yaw_from_quaternion(pose_.pose.orientation));

		// Check to see if we're within our limits
		if( (dp > goal_.position_radius) || (dy > goal_.yaw_range) )
			is_waiting = true; // Not yet, keep waiting
	} //Else we've reached then end and that's all!

	return is_waiting;
}

void Spar::cb_timer(const ros::TimerEvent& event) {
	check_start_as();

    // If we are running an action, then load in from our action
	// otherwise we're holding location, so don't touch output
    if ( as_.isActive() ) {
		//Marker to halt success until a further point in time
		bool is_waiting = false;

		// Alpha acts as our progress feedback, but is derived from our path progress
		//If we have a valid duration, then we set it as requried, otherwise set to 1.0 to imidiately finish
		double alpha = ( motion_duration_ > ros::Duration(0) ) ? ( ros::Time::now() - stamp_start_ ).toSec() / motion_duration_.toSec() : 1.0;
		if (alpha < 0.0)
			alpha = 0.0;
		if (alpha > 1.0)
			alpha = 1.0;

		switch(goal_.motion) {
			//All GOTO functions work in position-hold mode
			case spar_msgs::FlightMotionGoal::MOTION_GOTO_POS:
			case spar_msgs::FlightMotionGoal::MOTION_GOTO_YAW:
			case spar_msgs::FlightMotionGoal::MOTION_GOTO: {

				double dx = end_pos_.x - start_pos_.x;
				double dy = end_pos_.y - start_pos_.y;
				double dz = end_pos_.z - start_pos_.z;
				//double dyaw_a = shortest_angle(end_yaw_, start_yaw_);
				double dyaw = end_yaw_ - start_yaw_;

				output_.position.x = start_pos_.x + dx*alpha;
				output_.position.y = start_pos_.y + dy*alpha;
				output_.position.z = start_pos_.z + dz*alpha;
				output_.yaw = wrap_pi(start_yaw_ + dyaw*alpha);

				break;
			}
			//XXX: This is same as above, but without the extra math (as alpha == 1.0)
			case spar_msgs::FlightMotionGoal::MOTION_STOP: {
				output_.position = end_pos_;
				output_.yaw = wrap_pi(end_yaw_);

				break;
			}
			//TAKEOFF and LAND functions work in velocity-hold mode
			case spar_msgs::FlightMotionGoal::MOTION_TAKEOFF: {
				if(alpha < 1.0) {
					output_.velocity.x = 0.0;
					output_.velocity.y = 0.0;
					output_.velocity.z = goal_.velocity_vertical;

					output_.yaw = wrap_pi(end_yaw_);
				} else {
					if(!takeoff_complete_) {
						takeoff_complete_ = true;
						ROS_INFO("Take-off motion complete, switching to position hold");
					}
					// We're done the start-up part, switch to pos-hold and fly to location
					output_.type_mask = type_mask_from_motion(spar_msgs::FlightMotionGoal::MOTION_STOP);
					output_.position = end_pos_;
					output_.yaw = wrap_pi(end_yaw_);
				}

				break;
			}
			case spar_msgs::FlightMotionGoal::MOTION_LAND: {
				output_.velocity.x = 0.0;
				output_.velocity.y = 0.0;
				output_.velocity.z = -goal_.velocity_vertical;
				output_.yaw = wrap_pi(end_yaw_);

				//XXX:	Keep waiting forever!
				//		Disarm to finish landing sequence
				is_waiting = !landing_complete_;

				break;
			}
			default : {
				//Undefined behaviour
				//XXX: This is an internal check, don't rely on it for user interraction
				ROS_ASSERT_MSG(false, "[Spar] Undefined motion set: %i", goal_.motion);
			}
		}

		//Check our success cases (goal tracking has reached end-points)
		//XXX: This will happen imidiately if velocities are 0
		if (alpha >= 1.0) {
			//Check to see if we should be waiting (mux with other 'is_waiting' decisions)
			is_waiting = is_waiting || is_waiting_for_convergence();

			if(!is_waiting) {
				ROS_INFO("[Spar] %s action: Succeeded", param_action_name_.c_str());

				// Fill in details and set the action state to succeeded
				result_.final_position = output_.position;
				result_.final_yaw = output_.yaw;
				as_.setSucceeded(result_);

				//Reset the "special-case" variables
				takeoff_complete_ = false;
				landing_complete_ = false;
			}
		}

		//Output our feedback
		// XXX: Doesn't matter about sending it late,
		//      ROS does not make those guarentees anyway
		feedback_.goal_position = output_.position;
		feedback_.goal_yaw = output_.yaw;
		feedback_.progress = alpha;
		feedback_.waiting_for_convergence = is_waiting;
		as_.publishFeedback(feedback_);
	}

	//State change protection:
	if (is_running_) {
		bool do_stop = false;

		// Check in order of severity (don't flood output multiple cases)
		if(!mav_state_.connected) {
			do_stop = true;
			ROS_WARN("[Spar] Disconnect detected!");
		} else if(!mav_state_.armed) {
			if( (goal_.motion == spar_msgs::FlightMotionGoal::MOTION_LAND) && (!landing_complete_)) {
				//XXX:	Special case for landing mode (UAV should be disarmed mid-motion)
				//		This will buy us one more pass through the timer, which should
				//		then register the success case, and clear landing_complete_
				//		The outcome is that on the next pass, the typical "Disarm Detected" will run
				landing_complete_ = true;
			} else {
				//But in most cases this is a big issue (see: very unexpected)
				do_stop = true;
				ROS_WARN("[Spar] Disarm detected!");
			}
		} else if(mav_state_.mode != FLIGHT_MODE_OFFBOARD) {
			do_stop = true;
			ROS_WARN("[Spar] Mode change detected!");
		}

		// Our node is allowed to start back up when
		// we've gone back through the setup checks
		if(do_stop) {
			ROS_INFO("[Spar] Stopped action server!");
			cancel_goal();
			is_running_ = false;
			as_.shutdown();

			//Reset the "special-case" variables
			takeoff_complete_ = false;
			landing_complete_ = false;
		}
	} else {
		// We're not operating, so command to output to current location.
		// Kicking into offboard might trigger mid-flight might do something weird
		// or send us off to somewhere we're not expecting otherwise
		stop_at_current_location();
	}

	output_.header.stamp = ros::Time::now();
	pub_setpoint_.publish(output_);
}

}
