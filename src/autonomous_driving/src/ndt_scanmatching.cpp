#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include "autonomous_driving/ndt_original.h"

#include <chrono>

class ScanMatcher 
{
private:
    //プライベートメンバ変数
    ros::Subscriber lidar_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::TransformStamped ts_lis_, ts_bro_;
    //std::vector<NDT::NDTGrid> grids_;
    //std::vector<NDT::Point2D> points_map_;
    std::vector<custom_math::Matrix> points_odom_;
    NDT::NDTScanMatcher *ndt_scanmatcher_;

    //odom_to_map_->マップ座標系から見たオドメトリ座標系の位置と回転角度
    //robot_to_odom_->オドメトリ座標系から見たロボットの位置と回転角度
    custom_math::Matrix odom_to_map_;
    int nscan_data_ = 0;
    int matching_counter = 0;
    bool is_ready_ = false;

    //パラメータ
    int nskip_ = 0;
    int nskip_scan_ = 0; //何回おきのスキャンでマッチングを行うか
    double lidar_pos_[3] = {0};

    //sinとcosのテーブル
    double *table_sin_;
    double *table_cos_;

public:
    //50ms周期で実行される
    void lidarCb(const sensor_msgs::LaserScan::ConstPtr &scan_data)
    {
        if(matching_counter == nskip_scan_)
        {
            matching_counter = 0;
            std::chrono::system_clock::time_point t_now = std::chrono::system_clock::now();

            custom_math::Matrix R_odom(2, 2), trans_robot_to_odom(2, 1), scan_to_robot(2, 1);
            //custom_math::Matrix H = custom_math::Matrix::identity(3), I = custom_math::Matrix::identity(3);
            //custom_math::Matrix delta_p(3, 1), delta_grad(3, 1), grad_s(3, 1), grad_s_pre(3, 1);
            int table_index = 0;

            /*NDTを用いてマッチングを行う*/
            //オドメトリ座標系におけるロボット座標の取得
            try
            {
                ROS_INFO("scanmatcher: looking up tf from odom to base_link");
                ts_lis_ = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0));
            }
            catch(const tf2::TransformException& e)
            {
                //例外が発生した場合は、オドメトリ座標は更新されない
                ROS_WARN("%s", e.what());
            }
            //センサデータをオドメトリ座標系に変換するための回転行列と並進ベクトルを求める
            ROS_INFO("scanmatcher: calculating positions of points in odom frame");
            R_odom(0, 0) = ts_lis_.transform.rotation.w;
            R_odom(1, 0) = ts_lis_.transform.rotation.z;
            R_odom(0, 1) = -ts_lis_.transform.rotation.z;
            R_odom(1, 1) = ts_lis_.transform.rotation.w;
            R_odom = R_odom*R_odom;
            trans_robot_to_odom(0, 0) = ts_lis_.transform.translation.x;
            trans_robot_to_odom(1, 0) = ts_lis_.transform.translation.y;
            //custom_math::showMatrix(trans_robot_to_odom);

            for(int i=0; i<nscan_data_; i++)
            {
                //nskip個飛ばしでセンサデータを使用
                if(i%(nskip_+1) == 0)
                {
                    table_index = i/(nskip_+1);
                    //スキャンデータをロボット座標系に変換
                    scan_to_robot(0, 0) = scan_data->ranges[i]*table_cos_[table_index] + lidar_pos_[0];
                    scan_to_robot(1, 0) = scan_data->ranges[i]*table_sin_[table_index] + lidar_pos_[1];

                    //スキャンデータをオドメトリ座標系に変換
                    points_odom_[table_index] = R_odom*scan_to_robot + trans_robot_to_odom;
                }
            }
            //custom_math::showMatrix(points_odom_[0]);
            //スキャンデータをマップ座標系に変換し、対応するグリッドを求める
            //NDT::determineCorrespondingGrid(points_map_, points_odom_, odom_to_map_, grids_, 
            //    ngrids_, ngrids_col_, grid_size_, nscan_data_used_);

            /*準ニュートン法による最適化計算を行う*/
            ROS_INFO("scanmatcher: performing matching calculation");
            ndt_scanmatcher_->align(points_odom_, odom_to_map_);

            std::chrono::system_clock::time_point t_now2 = std::chrono::system_clock::now();
            std::cout << "scanmatcher: time required [ms]:" << std::chrono::duration_cast<std::chrono::microseconds>(t_now2 - t_now).count() << std::endl;
            //showMatrix(odom_to_map_);
            /*grad_s_pre = NDT::gradS(points_map_, points_odom_, odom_to_map_, nscan_data_used_);
            
            int criteria_counter = 1;
            while(1)
            {
                //パラメータ(マップ座標系とオドメトリ座標系の位置関係)の変更分を求める
                delta_p = H*grad_s*(-1.0);
                
                //パラメータの更新
                odom_to_map_ += delta_p;
                
                //更新されたパラメータに対し、再びgradを計算
                //もしかすると、determineCorrespondingGrid()は要らないかもしれない
                NDT::determineCorrespondingGrid(points_map_, points_odom_, odom_to_map_, grids_,
                    ngrids_, ngrids_col_, grid_size_ ,nscan_data_used_);
                grad_s = NDT::gradS(points_map_, points_odom_, odom_to_map_, nscan_data_used_);
                
                //gradの変化量が一定値以下になったor繰り返し回数が上限に達した場合は計算終了
                delta_grad = grad_s - grad_s_pre;
                if((delta_grad.T()*delta_grad)(0, 0) < criteria_eps_ || criteria_counter == criteria_n_)
                    break;
                criteria_counter++;
                
                //ヘシアンの近似逆行列の更新
                H = ( I - delta_grad*delta_p.T()*(1.0/(delta_grad.T()*delta_p)(0,0)) )
                    * H
                    * ( I - delta_grad*delta_p.T()*(1.0/(delta_grad.T()*delta_p)(0,0)) )
                    + delta_p*delta_p.T()*(1.0/(delta_grad.T()*delta_p)(0,0));

                grad_s_pre = grad_s;
            }*/

            //マップ座標系とオドメトリ座標系の間のTFをブロードキャスト
            ts_bro_.header.stamp = ros::Time::now();
            ts_bro_.header.frame_id = "map";
            ts_bro_.child_frame_id = "odom";
            ts_bro_.transform.translation.x = odom_to_map_(0, 0);
            ts_bro_.transform.translation.y = odom_to_map_(1, 0);
            ts_bro_.transform.rotation.z = sin(odom_to_map_(2, 0)*0.5);
            ts_bro_.transform.rotation.w = cos(odom_to_map_(2, 0)*0.5);
            tf_broadcaster_.sendTransform(ts_bro_);
        }
        else
        {
            matching_counter++;
        }
        
    }

    ScanMatcher(ros::NodeHandle &nh)
    : tf_listener_(tf_buffer_), odom_to_map_(3, 1)
    {
        int nscan_data_used = 0;
        int ngrids = 0;
        int ngrids_col = 0;  //横にいくつのグリッドがあるか
        double grid_size = 0;  //ひとつのグリッドのサイズ[m]
        double criteria_eps = 0, criteria_n = 0;  //準ニュートン法の終了条件

        /*パラメータの読み込み*/
        std::ifstream ifs_;
        std::string file_path;
        while(!nh.getParam("scanmatch/ndt_filepath", file_path));
        ifs_.open(file_path);

        if(ifs_)
        {
            //lambda関数
            auto splitTo5Double = [](const std::string &str, double *outputs) -> void
            {
                int i=0;
                std::string buf;

                for(auto itr = str.begin(); itr != str.end(); itr++)
                {
                    if(*itr == ' ')
                    {
                        //std::cout << buf << std::endl;
                        outputs[i] = std::stod(buf);
                        buf.clear();
                        i++;
                    }
                    else if(itr+1 == str.end())
                    {
                        buf.push_back(*itr);
                        //std::cout << buf << std::endl;
                        outputs[i] = std::stod(buf);
                        //buf.clear();
                        //i++;
                    }
                    else
                    {
                        buf.push_back(*itr);
                    }
                }        
            };

            ROS_INFO("scanmatcher: opened a file");

            /*パラメータの取得*/
            ROS_INFO("scanmatcher: fetching parameters...");
            while(!nh.getParam("scanmatch/grid_num", ngrids));
            while(!nh.getParam("scanmatch/grid_col_num", ngrids_col));
            while(!nh.getParam("scanmatch/scan_data_skip_num", nskip_));
            while(!nh.getParam("scanmatch/scan_skip_num", nskip_scan_));
            while(!nh.getParam("scanmatch/grid_size", grid_size));
            while(!nh.getParam("scanmatch/lidar_pos_x", lidar_pos_[0]));
            while(!nh.getParam("scanmatch/lidar_pos_y", lidar_pos_[1]));
            while(!nh.getParam("scanmatch/lidar_pos_th", lidar_pos_[2]));
            while(!nh.getParam("scanmatch/criteria_eps", criteria_eps));
            while(!nh.getParam("scanmatch/criteria_n", criteria_n));
            while(!nh.getParam("scanmatch/init_pos_x", odom_to_map_(0, 0)));
            while(!nh.getParam("scanmatch/init_pos_y", odom_to_map_(1, 0)));
            while(!nh.getParam("scanmatch/init_pos_th", odom_to_map_(2, 0)));
            ROS_INFO("scanmatcher: fetching parameters completed");
            
            /*ファイルを読み込んで、NDTのグリッドを作成する*/
            ROS_INFO("scanmatcher: creating ndt objects...");
            ndt_scanmatcher_ = new NDT::NDTScanMatcher(ngrids);
            ndt_scanmatcher_->setCriteria(criteria_eps, criteria_n);
            //grids_ = new NDT::NDTGrid*[ngrids_];
            
            std::string buf;
            double vals[5] = {0};
            for(int i=0; i<ngrids; i++)
            {
                //ファイルから一行読み込む
                std::getline(ifs_, buf);
                //一行を空白を区切り文字として5分割
                splitTo5Double(buf, vals);
                //gridを作成
                //grids_[i] = new NDT::NDTGrid(vals[0], vals[1], vals[2], vals[3], vals[4]);
                ndt_scanmatcher_->addGrid(i, vals[0], vals[1], vals[2], vals[3], vals[4]);
            }
            ROS_INFO("scanmatcher: completed creating ndt");
            //ndt_scanmatcher_->showGrid();

            //sin、cosのテーブルを作成（マッチングの計算に使う）
            ROS_INFO("scanmatcher: creating tables of sin and cos...");
            boost::shared_ptr<const sensor_msgs::LaserScan> scan_data = 
                ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan");
            

            nscan_data_ = scan_data->ranges.size();
            double scan_min_ang = scan_data->angle_min;
            //スキップされるデータ用の値は計算しない
            nscan_data_used = (nscan_data_-1)/(nskip_+1) + 1;
            points_odom_.resize(nscan_data_used);
            //points_map_.resize(nscan_data_used_);
            table_sin_ = new double[nscan_data_used];
            table_cos_ = new double[nscan_data_used];
            for(int i=0; i<nscan_data_; i++)
            {
                if(i%(nskip_+1) == 0)
                {
                    //points_odom_[i/(nskip_+1)] = custom_math::Matrix(2, 1);
                    table_sin_[i/(nskip_+1)] = sin(lidar_pos_[2] + scan_min_ang + M_PI/720.0*i);
                    table_cos_[i/(nskip_+1)] = cos(lidar_pos_[2] + scan_min_ang + M_PI/720.0*i);
                }
            }

            ndt_scanmatcher_->setGridAndPointInfo(nscan_data_used, ngrids_col, grid_size);
            ROS_INFO("scanmatcher: creating tables of sin and cos completed");

            //サブスクライバの作成
            lidar_sub_ = nh.subscribe("scan", 1, &ScanMatcher::lidarCb, this);
            //行列計算で用いる一時領域を確保
            custom_math::Matrix::allocTmpSpace(3, 3);
            is_ready_ = true;
        }
        else
        {
            ROS_INFO("scanmatcher: could not open a file");
            is_ready_ = false;
        }

        ifs_.close();
    }

    ~ScanMatcher()
    {
        custom_math::Matrix::freeTmpSpace();
        delete[] table_sin_;
        delete[] table_cos_;
        delete ndt_scanmatcher_;
    }

    void start()
    {
        if(is_ready_)
        {
            //base_linkからodomのtfが出ているかチェック
            while(!tf_buffer_.canTransform("odom", "base_link", ros::Time(0), ros::Duration(5.0)))
            {
                ROS_WARN("scanmatcher: could not get transform from base_link to odom");
            }

            
            ROS_WARN("scanmatcher: published the first tf from map to odom");
            
            //mapからodomのtfの初期値を出しておく
            ts_bro_.header.stamp = ros::Time::now();
            ts_bro_.header.frame_id = "map";
            ts_bro_.child_frame_id = "odom";
            ts_bro_.transform.translation.x = odom_to_map_(0, 0);
            ts_bro_.transform.translation.y = odom_to_map_(1, 0);
            ts_bro_.transform.translation.z = 0;
            ts_bro_.transform.rotation.x = 0;
            ts_bro_.transform.rotation.y = 0;
            ts_bro_.transform.rotation.z = sin(odom_to_map_(2, 0)*0.5);
            ts_bro_.transform.rotation.w = cos(odom_to_map_(2, 0)*0.5);
            tf_broadcaster_.sendTransform(ts_bro_);

            ros::spin();
        }
        else
        {
            ROS_INFO("scanmatcher: shutting down this node...");
        }
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ndt_scanmatcher");
    ros::NodeHandle nh;
    ScanMatcher sm(nh);
    
    sm.start();

    return 0;
}