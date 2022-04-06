//足回りの指令に使うROSメッセージ型が定義されたヘッダをインクルードする
#include "abu2022_msgs/BaseCmd.h"

//足回りの指令に使うROSメッセージ型を定義する
#define BASE_CMD_MSG_TYPE abu2022_msgs::BaseCmd

//足回りの指令メッセージに値を詰める関数を定義する
namespace base_cmd
{
    void setValue(BASE_CMD_MSG_TYPE &msg, double vx, double vy, double omega)
    {
        msg.omega = omega; 
        msg.vx = vx;
        msg.vy = vy;
    }
}

