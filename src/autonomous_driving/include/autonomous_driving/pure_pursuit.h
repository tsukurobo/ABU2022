#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include "base_msg_type.h"

// class Point{
// 	public:
// 		double x;
// 		double y;

// 		Point(double x=0, double y=0);
// 		Point(const Point& point);
// 		double change_to(double x, double y);
// 		double dist(double x, double y);
// 		double dist(Point point);
// 		double print();
// };

// class Pure_pursuit{
// 	public:
// 		//状態
// 		std::vector<Point> path; //経路点列
// 		Point state_p; //ロボ位置([m],[m])
// 		double state_yaw; //ロボ姿勢[rad]

// 		//司令変数
// 		double cmd_vx;
// 		double cmd_vy;
// 		double cmd_w;

// 		//関数
// 		/* Pure_pursuit(std::string file_name, int ahead_num); //コンストラクタ */
// 		void reset_path(std::string file_name, int ahead_num); //経路再設定
// 		void set_state(Point pos, double yaw);          //ロボ状態設定
// 		void set_state(double x, double y, double yaw); //ロボ状態設定
// 		void set_position(Point pos);          //ロボ位置設定
// 		void set_position(double x, double y); //ロボ位置設定
// 		void set_posture(double yaw); //ロボ姿勢設定
// 		void cmd_angular_v(double p, double i, double d); //角速度司令[rad/s]
// 		double cmd_velocity(double speed, double fin, double dcl); //速度司令[m/s]
// 		int print_path(); //経路点列表示
	
// 	private:
// 		//パラメータ
// 		std::string file_name; //読み込むCSVファイル名
// 		int ahead_num; //最近経路点からahead_num個先の点を目標点とする
		
// 		//関数
// 		double dist_fin(); //経路点列終端との距離[m]
// 		double target_dir_local(); //目標角度[rad](ローカル角度)
// 		double target_dir_global(); //目標角度[rad](グローバル角度)
// 		Point target_point(); //目標点
// 		int load_csv(); //CSVフィアル読み込み
// };


// /************************************************************/
// /********** Point class *************************************/
// /************************************************************/

// //コンストラクタ
// Point::Point(double x, double y){
// 	this->x = x;
// 	this->y = y;
// }

// //コピーコンストラクタ（vector<Point>用）
// Point::Point(const Point& point){
// 	this->x = point.x;
// 	this->y = point.y;
// }

// //座標変化
// double Point::change_to(double x, double y){
// 	this->x = x;
// 	this->y = y;
// }

// //2点間距離
// double Point::dist(double x, double y){
// 	return sqrt(pow(x - this->x, 2) + pow(y - this->y, 2));
// }
// double Point::dist(Point point){
// 	return sqrt(pow(point.x - this->x, 2) + pow(point.y - this->y, 2));
// }

// //表示
// double Point::print(){
// 	printf("x:%f\ty:%f\n", x, y);
// }




// /*******************************************************************/
// /********** Pure_pursuit class *************************************/
// /*******************************************************************/

// /******* 設定 *******/
// //コンストラクタ
// /* Pure_pursuit::Pure_pursuit(std::string file_name, int ahead_num){ */
// /* 	this->file_name = file_name; */
// /* 	this->ahead_num = ahead_num; */

// /* 	load_csv(); */
// /* } */

// //経路再設定
// void Pure_pursuit::reset_path(std::string file_name, int ahead_num){
// 	this->file_name = file_name;
// 	this->ahead_num = ahead_num;

// 	load_csv();
// }

// //ロボ状態設定
// void Pure_pursuit::set_state(Point pos, double yaw){
// 	this->state_p = pos;
// 	this->state_yaw = yaw;
// }
// void Pure_pursuit::set_state(double x, double y, double yaw){
// 	this->state_p.x = x;
// 	this->state_p.y = y;
// 	this->state_yaw = yaw;
// }

// //ロボ位置設定
// void Pure_pursuit::set_position(Point pos){
// 	this->state_p = pos;
// }
// void Pure_pursuit::set_position(double x, double y){
// 	this->state_p.x = x;
// 	this->state_p.y = y;
// }

// //ロボ姿勢設定
// void Pure_pursuit::set_posture(double yaw){
// 	this->state_yaw = yaw;
// }

// /******* 司令 *******/
// //角速度司令[rad/s]
// void Pure_pursuit::cmd_angular_v(double p, double i, double d){
// 	double angle = target_dir_global() - state_yaw; //姿勢角と目標角との偏角
// 	static double sum_yaw = 0;
// 	static double pre_yaw = 0;

// 	//偏角定義域修正
// 	if(angle > M_PI)       while(angle >  M_PI) angle -= 2*M_PI;	
// 	else if(angle < -M_PI) while(angle < -M_PI) angle += 2*M_PI;

// 	sum_yaw += angle;
// 	pre_yaw = state_yaw;
// 	/* ROS_FATAL("\nstate_yaw: %f\ttrgt: %f\tangle: %f", state_yaw/M_PI*180, target_dir_global()/M_PI*180, angle/M_PI*180); */

// 	//PID制御
// 	cmd_w = p*angle + i*sum_yaw - d*(state_yaw - pre_yaw);
// }

// //速度司令[m/s]
// double Pure_pursuit::cmd_velocity(double max_speed, double fin, double dcl){
// 	double speed; //速さ（スカラー）

// 	//終端点との距離に応じ速さ変更
// 	if(dist_fin() < fin)      speed = 0; //終了判定
// 	else if(dist_fin() < dcl) speed = max_speed * (dist_fin() - fin) / (dcl - fin); //台形制御
// 	else                      speed = max_speed;

// 	cmd_vx = speed * cos(target_dir_local());
// 	cmd_vy = speed * sin(target_dir_local());

// 	return speed;
// }

// //経路点列終端との距離
// double Pure_pursuit::dist_fin(){
// 	return state_p.dist(path.back());
// }


// /******* 目標角度 *******/
// //目標角度[rad](ローカル角度)
// double Pure_pursuit::target_dir_local(){
// 	return target_dir_global() - state_yaw;
// }

// //目標角度[rad](グローバル角度)
// double Pure_pursuit::target_dir_global(){
// 	Point trgt = target_point(); //目標点
// 	return atan2(trgt.y - state_p.y, trgt.x - state_p.x); //偏角[rad](グローバル角度)
// }

// //目標点
// Point Pure_pursuit::target_point(){
// 	std::vector<double> dist; //各点との距離

// 	//距離計算
// 	for(int i=0; i<path.size(); i++){
// 		/* dist.push_back(path->at(i).dist(state_p)); */
// 		dist.push_back(state_p.dist(path.at(i)));
// 	}
// 	//最近経路点
// 	std::vector<double>::iterator itr = std::min_element(dist.begin(), dist.end());
// 	int index = std::distance(dist.begin(), itr);
// 	//最近経路点からahead_num個先の点を目標点とする
// 	if(path.size() > index+ahead_num){
// 		return path.at(index + ahead_num);
// 	}else{
// 		return path.back();
// 	}
// }

// /******* デバッグ *******/
// //経路点列表示
// int Pure_pursuit::print_path(){
// 	for(int i=0; i<path.size(); i++) path.at(i).print();
// }

// /******* CSVファイル読み込み *******/
// //CSVファイル読み込み
// int Pure_pursuit::load_csv(){
// 	std::ifstream file(file_name);

// 	// check file open
// 	if(file.fail()){
// 		ROS_FATAL("Failed to open file\n");
// 		return -1;
// 	}

// 	// get file
// 	path.clear();

// 	while(1){
// 		std::string line_x;
// 		std::string line_y;

// 		//取得
// 		getline(file, line_x, ',');
// 		getline(file, line_y);

// 		//ファイル終端判定
// 		if(file.eof()) break;

// 		//pathに追加
// 		Point tmp(stod(line_x), stod(line_y));
// 		path.push_back(tmp);
// 	}

// 	file.close();

// 	return 0;
// }

namespace pure_pursuit
{
	class PathContainer
	{
	private:
		nav_msgs::Path path_;
		std::vector<double> dists_;

	public:
		PathContainer(){}

		void setPath(const nav_msgs::Path &);
		int getWaypointNum();
		double getSpeedCmd(int);
		double getRotCmd(int);
		double getWaypointPosX(int);
		double getWaypointPosY(int);
		int getNearestPointIndex(geometry_msgs::Vector3, int);
		void calcDirectionToWaypoint(geometry_msgs::Transform, int, double &, double &);
	};

	void PathContainer::setPath(const nav_msgs::Path &path)
	{
		path_ = path;
		dists_.resize(path_.poses.size());
	}

	int PathContainer::getWaypointNum()
	{
		return path_.poses.size();
	}

	double PathContainer::getSpeedCmd(int index)
	{
		return path_.poses[index].pose.orientation.x;
	}

	double PathContainer::getRotCmd(int index)
	{
		return path_.poses[index].pose.orientation.y;
	}

	double PathContainer::getWaypointPosX(int index)
	{
		return path_.poses[index].pose.position.x;
	}

	double PathContainer::getWaypointPosY(int index)
	{
		return path_.poses[index].pose.position.y;
	}

	int PathContainer::getNearestPointIndex(geometry_msgs::Vector3 curr_pos, int start_index)
	{
		int dists_index = 0;
		double delta_x = 0, delta_y = 0;
		
		if(start_index < path_.poses.size()-1)
		{
			for(auto itr = dists_.begin(); itr != dists_.end(); itr++)
				*itr = 1.0e12;
			
			for(int i = start_index; i < path_.poses.size(); i++)
			{
				/* dist.push_back(path->at(i).dist(state_p)); */
				delta_x = path_.poses[i].pose.position.x - curr_pos.x;
				delta_y = path_.poses[i].pose.position.y - curr_pos.y;
				dists_[dists_index] = delta_x*delta_x + delta_y*delta_y;
				dists_index++;
			}
			
			std::vector<double>::iterator min_elem = std::min_element(dists_.begin(), dists_.end());

			// std::cout << "dists_: " << std::endl;
			// for (double x : dists_)
			// {
			// 	std::cout << x << " ";
			// }
			// std::cout << std::endl;
			//std::cout << curr_pos.x << " " << curr_pos.y << std::endl;
			
			return (start_index + std::distance(dists_.begin(), min_elem));
		}
		else
		{
			return path_.poses.size()-1;
		}
	}

	void PathContainer::calcDirectionToWaypoint(geometry_msgs::Transform t, int wp_i, double &dir_x, double &dir_y)
	{
		double delta_x = path_.poses[wp_i].pose.position.x - t.translation.x,
			   delta_y = path_.poses[wp_i].pose.position.y - t.translation.y,
			   dist_to_wp = sqrt(delta_x*delta_x + delta_y*delta_y),

			   sin_theta_wp = 
				(path_.poses[wp_i].pose.position.y - t.translation.y)/dist_to_wp,
			   cos_theta_wp = 
				(path_.poses[wp_i].pose.position.x - t.translation.x)/dist_to_wp,
		
			   sin_theta_robot_half = t.rotation.z,
			   cos_theta_robot_half = t.rotation.w,
			   sin_theta_robot = 2.0*sin_theta_robot_half*cos_theta_robot_half,
			   cos_theta_robot = 
			   	cos_theta_robot_half*cos_theta_robot_half - sin_theta_robot_half*sin_theta_robot_half,

			   sin_theta_robot_wp = 
				sin_theta_wp*cos_theta_robot - cos_theta_wp*sin_theta_robot,
			   cos_theta_robot_wp = 
			    cos_theta_wp*cos_theta_robot + sin_theta_wp*sin_theta_robot;

		dir_x = cos_theta_robot_wp;
		dir_y = sin_theta_robot_wp;
	}

}