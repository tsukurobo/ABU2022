//データ解析用
#include <ros/ros.h>
#include <abu2022_msgs/BaseData.h>

class OmniDataCollector2
{
private:
    ros::Subscriber data_sub_;
    float *data_buf_;
    int measure_num_;
    int measure_counter_;

public:
    void dataCb(const abu2022_msgs::BaseData::ConstPtr &msg)
    {
        if(measure_counter_ < measure_num_)
        {       
            data_buf_[measure_counter_] = msg->theta;
            measure_counter_++;

            if(measure_counter_ == measure_num_)
            {
                float sum = 0;
                
                for(int i=0; i<measure_num_; i++) sum += data_buf_[i];
                ROS_INFO("drift is %.3f", sum/measure_num_);
            }
        }       
    }

    void startSpin()
    {
        ros::spin();
    }

    OmniDataCollector2(ros::NodeHandle &nh, int n)
    : measure_num_(n), measure_counter_(0)
    {
        data_sub_ = nh.subscribe("omni_info", 100, &OmniDataCollector2::dataCb, this);
        
        data_buf_ = new float[measure_num_];
    }

    ~OmniDataCollector2()
    {
        delete data_buf_;
    }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_commander");
    ros::NodeHandle nh;
    OmniDataCollector2 dc(nh, atoi(argv[1]));

    dc.startSpin();    
}
