#include "autonomous_driving/pure_pursuit.h"

namespace pure_pursuit
{
	class PathTracker
	{
	private:
		class PICalculator
		{
		private:
			double e_sum_ = 0,
				   kp_ = 0,
				   ki_ = 0,
				   max_output_ = 0;

		public:
			PICalculator(){}

			void setParams(double kp, double ki, double max_v)
			{
				kp_ = kp;
				ki_ = ki;
				max_output_ = max_v;
			}

			void reset()
			{
				e_sum_ = 0;
			}

			double calcOutput(double current_e)
			{
				e_sum_ += current_e;
				double result = kp_*current_e + ki_*e_sum_;

				if(result < -max_output_)
				{
					result = -max_output_;
					e_sum_ -= current_e;
				}
				else if(result > max_output_)
				{
					result = max_output_;
					e_sum_ -= current_e;
				}

				return result;
			}
		};

		ros::Timer timer_;
		ros::Publisher cmd_pub_;
		ros::Subscriber path_sub_;
		ros::Subscriber ctrl_cmd_sub_;
		PathContainer current_path_;
		tf2_ros::TransformListener tf_listener_;
		tf2_ros::Buffer tf_buffer_;
		geometry_msgs::TransformStamped ts_;
		BASE_CMD_MSG_TYPE base_cmd_;
		
		int last_nearest_point_index_;
		PICalculator pic_yaw_, pic_x_, pic_y_;
		enum TrackingMode
		{
			PURE_PURSUIT,
			PI_CONTROL
		}
		tracking_flag_;
		double theta_goal_;

		//parameters
		int ahead_num_;
		double eps_;

	public:
		PathTracker(ros::NodeHandle &nh)
		: tf_listener_(tf_buffer_)
		{
			//fetch parameters
			std::string cmd_topic_name;
			double timer_dt = 0, 
				   yaw_pid_kp = 0, yaw_pid_ki = 0, yaw_pid_max = 0,
				   pos_pid_kp = 0, pos_pid_ki = 0, pos_pid_max = 0;

			while(!nh.getParam("pure_pursuit/cmd_topic_name", cmd_topic_name)){ROS_ERROR("could not get param: cmd_topic_name");}
			while(!nh.getParam("pure_pursuit/control_loop_period", timer_dt)){ROS_ERROR("could not get param: control_loop_period");}
			while(!nh.getParam("pure_pursuit/ahead_num", ahead_num_)){ROS_ERROR("could not get param: ahead_num");}
			while(!nh.getParam("pure_pursuit/yaw_pid_kp", yaw_pid_kp)){ROS_ERROR("could not get param: yaw_pid_kp");}
			while(!nh.getParam("pure_pursuit/yaw_pid_ki", yaw_pid_ki)){ROS_ERROR("could not get param: yaw_pid_ki");}
			while(!nh.getParam("pure_pursuit/yaw_pid_max_val", yaw_pid_max)){ROS_ERROR("could not get param: yaw_pid_max_val");}
			while(!nh.getParam("pure_pursuit/pos_pid_kp", pos_pid_kp)){ROS_ERROR("could not get param: pos_pid_kp");}
			while(!nh.getParam("pure_pursuit/pos_pid_ki", pos_pid_ki)){ROS_ERROR("could not get param: pos_pid_ki");}
			while(!nh.getParam("pure_pursuit/pos_pid_max_val", pos_pid_max)){ROS_ERROR("could not get param: pos_pid_max_val");}
			while(!nh.getParam("pure_pursuit/pos_pid_eps", eps_)){ROS_ERROR("could not get param: pos_pid_eps");}

			timer_ = nh.createTimer(ros::Duration(timer_dt), &PathTracker::performControlCalculation, this);
			//the callback function is not called until a call of start()
			timer_.stop();
			cmd_pub_ = nh.advertise<BASE_CMD_MSG_TYPE>(cmd_topic_name, 10);
			path_sub_ = nh.subscribe("path", 1, &PathTracker::onPathReceived, this);
			ctrl_cmd_sub_ = nh.subscribe("enable_tracking", 1, &PathTracker::onEnableFlagReceived, this);
			pic_yaw_.setParams(yaw_pid_kp, yaw_pid_ki, yaw_pid_max);
			pic_x_.setParams(pos_pid_kp, pos_pid_ki, pos_pid_max);
			pic_y_.setParams(pos_pid_kp, pos_pid_ki, pos_pid_max);
		}

		//set path
		void onPathReceived(const nav_msgs::Path::ConstPtr &path)
		{
			current_path_.setPath(*path);
			last_nearest_point_index_ = 0;

			ROS_INFO("pure_pursuit_tracker: received a new path");
			//ROS_INFO("pure_pursuit_tracker: waypoint num: %d", current_path_.getWaypointNum());
		}

		//start or stop tracking
		void onEnableFlagReceived(const std_msgs::Bool::ConstPtr &cmd)
		{
			if(cmd->data == true)
			{
				if(current_path_.getWaypointNum() == 0)
				{
					ROS_WARN("pure_pursuit_tracker: set a path before starting tracking");
					return;
				}

				tracking_flag_ = PURE_PURSUIT;
				pic_yaw_.reset();
				timer_.start();
			}
			else if(cmd->data == false)
			{
				timer_.stop();
				//stop the robot
				base_cmd::setValue(base_cmd_, 0, 0, 0);
				cmd_pub_.publish(base_cmd_);
			}
		}

		//make the direction to which the robot should head
		void performControlCalculation(const ros::TimerEvent &e)
		{
			try
			{
				//fetch current position of the robot
				ts_ = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
				geometry_msgs::Vector3 current_pos = ts_.transform.translation;
				double theta_robot = 2.0*atan2(ts_.transform.rotation.z, ts_.transform.rotation.w),
					   cmd_vx, cmd_vy;

				if(tracking_flag_ == PURE_PURSUIT)
				{
					//find the nearest waypoint from the robot
					last_nearest_point_index_ = current_path_.getNearestPointIndex(current_pos, 
						last_nearest_point_index_);

					int look_ahead_point_index = last_nearest_point_index_ + ahead_num_;
					if(look_ahead_point_index >= current_path_.getWaypointNum())
						look_ahead_point_index = current_path_.getWaypointNum()-1;

					if(last_nearest_point_index_ == current_path_.getWaypointNum()-1)
					{
						tracking_flag_ = PI_CONTROL;
						pic_x_.reset();
						pic_y_.reset();

						ROS_INFO("pure_pursuit_tracker: position control has started");
					}

					theta_goal_ = current_path_.getRotCmd(last_nearest_point_index_);

					//get target speed and angle from the nearest waypoint
					double speed_goal = current_path_.getSpeedCmd(last_nearest_point_index_),
						/*theta_goal = current_path_.getRotCmd(last_nearest_point_index_),
						theta_robot = 2.0*atan2(ts_.transform.rotation.z, ts_.transform.rotation.w),*/
						dir_to_lap_x = 0,
						dir_to_lap_y = 0;

					//calc direction of look ahead point in robot frame
					current_path_.calcDirectionToWaypoint(ts_.transform, look_ahead_point_index, 
						dir_to_lap_x, dir_to_lap_y); 

					//make a command
					cmd_vx = speed_goal*dir_to_lap_x,
					cmd_vy = speed_goal*dir_to_lap_y;
				}
				else if(tracking_flag_ == PI_CONTROL)
				{
					double pos_x_error = current_path_.getWaypointPosX(last_nearest_point_index_) - current_pos.x,
						   pos_y_error = current_path_.getWaypointPosY(last_nearest_point_index_) - current_pos.y;
					
					if(pos_x_error*pos_x_error + pos_y_error*pos_y_error > eps_)
					{
						cmd_vx = pic_x_.calcOutput(pos_x_error);
						cmd_vy = pic_y_.calcOutput(pos_y_error);
					}
					else
					{
						cmd_vx = 0;
						cmd_vy = 0;
						
						base_cmd::setValue(base_cmd_, cmd_vx, cmd_vy, 0);
						cmd_pub_.publish(base_cmd_);
						
						ROS_INFO("pure_pursuit_tracker: path tracking has finished");
						timer_.stop();

						return;
					}
				}
				double	angle_error = theta_goal_ - theta_robot;
				
				if(angle_error > M_PI)       while(angle_error >  M_PI) angle_error -= 2*M_PI;	
				else if(angle_error < -M_PI) while(angle_error < -M_PI) angle_error += 2*M_PI;

				double cmd_w = pic_yaw_.calcOutput(angle_error);

				//publish a command
				base_cmd::setValue(base_cmd_, cmd_vx, cmd_vy, cmd_w);
				cmd_pub_.publish(base_cmd_);
			}
			catch(const tf2::TransformException& e)
			{
				ROS_ERROR("%s", e.what());
				// ROS_ERROR("pure_pursuit_tracker: a stop command has sent to the robot");
				// //stop the robot
				// base_cmd::setValue(base_cmd_, 0, 0, 0);
				// cmd_pub_.publish(base_cmd_);
			}
			
		}

		void startTracking()
		{
			ros::spin();
		}

	};
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pp_path_tracker");
	ros::NodeHandle nh;

	pure_pursuit::PathTracker pt(nh);
	pt.startTracking();

	return 0;
}
