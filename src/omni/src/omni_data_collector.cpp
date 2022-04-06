//データ解析用
#include <ros/ros.h>
#include <abu2022_msgs/BaseData.h>
#include <fstream>
#include <math.h>

class OmniDataCollector
{
private:
    ros::Subscriber data_sub_;
    std::ofstream ofs_;
    bool is_file_opened_;

public:
    void dataCb(const abu2022_msgs::BaseData::ConstPtr &msg)
    {
        ofs_ << msg->theta << "," << msg->e1*6*0.56 << std::endl; 
    }

    void startSpin()
    {
        /*while(ros::ok())
        {
            ros::spinOnce();

            y = goal_vel;
            for(int i=0; i<N-1; i++) y += x[i];
            y /= (double)N;
            for(int i=0; i<N-1; i++) x[i] = (i == N-2) ? goal_vel : x[i+1];
            //y = (goal_vel + x[1] + x[0])/3.0;
            //x[0] = x[1]; x[1] = goal_vel;

            d.data = y;
            data_pub.publish(d);
            loop_rate_.sleep();
        }*/
        ros::spin();
    }

    OmniDataCollector(ros::NodeHandle &nh, char *file_path)
    {
        data_sub_ = nh.subscribe("omni_info", 100, &OmniDataCollector::dataCb, this);
        
        ofs_.open(file_path, std::ios::app);
        if(ofs_) is_file_opened_ = true;
        else is_file_opened_ = false;
    }

    ~OmniDataCollector()
    {
        if(is_file_opened_) ofs_.close();
    }

    /*~OmniCommander()
    {
        delete loop_rate_;
    }*/
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_commander");
    ros::NodeHandle nh;
    OmniDataCollector dc(nh, argv[1]);

    dc.startSpin();    
}
