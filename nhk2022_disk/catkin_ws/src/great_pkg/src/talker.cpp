#include "ros/ros.h" /* ROSの関数を使うのに必須 */
#include "std_msgs/String.h" /* Messageの型の定義 */

#include <sstream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");/* Nodeを初期化。第３引数はノードの名前 */

    ros::NodeHandle n;/* PublisherとかSUbscriberとか扱う */

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
// 型　実態
// <メッセージタイプ>(名前, バッファだかのサイズ)

// ループの周波数　10HZ分のデータを持った変数を作る
    ros::Rate loop_rate(10);


    int count = 0;
    while (ros::ok())/* ノードが動いてる間 */
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();/* 一回 */

        loop_rate.sleep();/* 10HZになるようにうまく待ち時間をつくる */
        ++count;
    }
    return 0;
}
