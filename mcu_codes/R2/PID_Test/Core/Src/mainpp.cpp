//可読性と移植性を考慮(した結果色々複雑になった泣)
//何をやっていたのか分からないが、RobotBaseクラスは薄いラッパーになってしまった
//IMUの操作の部分は分離する（負け）
//To do: パラメータをROS側から設定できるようにする、コードの(大幅な)リファクタリング（何回目だよ）

#include <ros.h>
//#include <ros/time.h>
#include <abu2022_msgs/BaseCmd.h>
#include <abu2022_msgs/BaseData.h>
//#include <tf/transform_broadcaster.h>
#include <stdlib.h>
#include "main.h"
#include "RobotBase.h"
#include "omni4_gyro_smd.h"
#include "math.h"

//基本的にいじらない定数とマクロ
#define ZERO_TH 1.0E-10
#define __IS_FLOAT_ZERO(x) ((x<ZERO_TH) && (x>-ZERO_TH))
#define __FLOAT_SIGN(x) ((x>-ZERO_TH) ? 1.0 : -1.0)

//#define RECEIVE_DATA_NUM 3 //PCから指令値として送られてくる値の数
#define VX 0
#define VY 1
#define OMEGA 2
#define OMEGA_WHEEL_REF_ACC 0
#define OMEGA_WHEEL_REF 1
#define OMEGA_WHEEL 2

//IMU用定数
#define CONV_TO_RAD 0.0174533 //pi/180
#define INV_ADC_RESOL 0.00024420024 // = 1/4095
#define MPU_ADDR (0x68<<1)
#define MPU_WHO_AM_I 0x75 //MPU6050のWHO_AM_Iレジスタのアドレス（レジスタに0x68が格納されていればok）
#define MPU_PWR_MGMT_1 0x6B //MPU6050のpower management 1レジスタのアドレス
#define MPU_CONFIG 0x1A //MPU6050のconfigurationレジスタのアドレス
#define MPU_GYRO_CONFIG 0x1B //MPU6050のgyroscope configurationレジスタのアドレス
#define MPU_GYRO_Z 0x47 //MPU6050のGYRO_ZOUT_Hのアドレス
#define MPU_GYRO_RANGE 1000.0 //単位は°/s
const float CONV_TO_DEG_PER_SEC = MPU_GYRO_RANGE/INT16_MAX;

//機構と制御のパラメータ
//#define MOTOR_NUM 4
#define BASE_TYPE Omni4 //足回りの構成
#define DT_CURRENT_PID 0.0002 //電流制御ループの周期 [s]
#define DT_VEL_PID 0.002 //速度制御ループの周期 [s]
#define DT_MAIN 0.01 //メインループの周期 [s]
#define ENC_RESOL 8192.0 //エンコーダの分解能 [pulse/回転]
#define Ks_ENC 0.4 //エンコーダカウンタの平滑定数
#define WHEEL_RADIUS 0.05 //オムニホイールの半径 [m]
#define DIST_TO_CENTER 0.3 //オムニの中心-ホイール間の距離 [m]
#define ROT_START_CURRENT 11.6 //最大静止摩擦抵抗を打ち消すための電流 [A]
#define FRIC_COMP_C1 5.0 //摩擦補償用定数1
//#define ROT_START_CURRENT 0.0 //最大静止摩擦抵抗を打ち消すための電流 [A]
//#define FRIC_COMP_C1 0.0 //摩擦補償用定数1
//#define FRIC_COMP_C2 1.0 //摩擦補償用定数2
#define INV_Kt 7.143 //モーターのトルク係数の逆数 [A/(N*m)]
#define J 0.00905 //オムニホイールの慣性モーメント [kg*m^2]
//#define J 0.0 //オムニホイールの慣性モーメント [kg*m^2]
#define Ks_CURRENT 0.3 //電流センサの値の平滑定数
const float CONV_ENCD_TO_OMEGA = 2.0*3.141592/ENC_RESOL/DT_VEL_PID, //エンコーダカウンタ変化量から角速度を求める定数
			CONV_ENCD_TO_DISP = 2.0*3.141592*WHEEL_RADIUS/ENC_RESOL,  //エンコーダカウンタ変化量から移動量を求める定数
			INV_WHEEL_RADIUS = 1.0/WHEEL_RADIUS,  //オムニホイールの半径の逆数
			INV_DIST_TO_CENTER = 1.0/DIST_TO_CENTER,	//DIST_TO_CENTERの逆数
			INV_DT_MAIN = 1.0/DT_MAIN; //メインループの周期の逆数

const float J_DIV_Kt = J*INV_Kt, b_DIV_Kt = 0.0145*INV_Kt, Fo = 1.654, Kt_DIV_Jomni = 0.14/0.56;

//#define N_FILTER 20 //移動平均フィルタのサンプル数
#define Kp_YAW_FIX 1.0 //回転角度修正比例制御用定数
#define OMEGA_FIX_TH 0.2 //これよりも速い角速度[rad/s]に対して回転角度修正がかかる
#define ACC_VEL 6.0 //並進速度の加速度 [m/s^2]
#define ACC_ROT 18.85 //回転角速度の加速度 [rad/s^2]
#define VEL_PID_Kp 3.0 //速度制御PIDコントローラのPゲイン
#define VEL_PID_Ki 30.0 //速度制御PIDコントローラのIゲイン
#define VEL_PID_Kd 0.0 //速度制御PIDコントローラのDゲイン
#define CURR_MAX 35.0 //各モーターに流す電流の最大値 [A]
#define VEL_PID_MODE BUMPLESS //速度制御PIDコントローラの計算方式
#define CURR_PID_Kp 300.0 //電流制御PIDコントローラのPゲイン
//#define CURR_PID_Kp 0.0 //電流制御PIDコントローラのPゲイン
#define CURR_PID_Ki 50000.0 //電流制御PIDコントローラのIゲイン
//#define CURR_PID_Ki 100.0 //電流制御PIDコントローラのIゲイン
#define CURR_PID_Kd 0.0 //電流制御PIDコントローラのDゲイン
#define ARR_MAX 4199.0 //ARRの最大値
#define CURR_PID_MODE BUMPLESS //電流制御PIDコントローラの計算方式
#define IMU_TIMEOUT_TH 100
//#define IMU_I2C_ERROR_DETECT_TH 20.0 //IMUの現在の計測値と前回の計測値の差の絶対値がこれよりも大きいと、通信エラーが起きているとみなす
//#define IMU_MALFUNC_DETECT_CNT_TH 100 //通信エラーがこの回数検出されると、IMUは誤動作しているとみなす

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern ADC_HandleTypeDef hadc1;
//extern I2C_HandleTypeDef hi2c1;
extern IWDG_HandleTypeDef hiwdg;
//UART_HandleTypeDef huart2;


/**
  * @brief  This funciton performs setting a goal.
  * @param  a goal velocity which includes vx, vy, and omega
  * @retval None
  */
//void VelCmdCb(const std_msgs::Float32MultiArray &data){
//	for(int i=0; i<RECEIVE_DATA_NUM; i++) goal_vel[i] = data.data[i];
//}

///*グローバル変数定義*///
//baseハンドラとPIDコントローラ
RobotBase<BASE_TYPE> *omni4;
PIDCalculator **current_pid_controller, **vel_pid_controller;

//メインループ用変数
//float goal_vel_pre[RECEIVE_DATA_NUM][N_FILTER - 1] = {0};
float goal_vel_smoothed[RECEIVE_DATA_NUM] = {0}, goal_vel_smoothed_pre[RECEIVE_DATA_NUM] = {0};
float motor_goal_vel_pre[BASE_TYPE::MOTOR_NUM] = {0}, motor_goal_vel_acc[BASE_TYPE::MOTOR_NUM] = {0};

//IMU用変数
//そしてオドメトリ用変数
//uint8_t imu_data[2] = {0};
float /*gyro_z = 0,*/ omni_delta_theta = 0, omni_delta_x = 0, omni_delta_y = 0;
//unsigned int timeout_counter = 0; //imu_malfunc_counter = 0;
//bool is_imu_ok = false, imu_data_request_flag = false;

//速度制御中のFF制御で使う関数
float FFFunc (float *vals)
{
	  //const float J = 1.0, inv_Kt = 1.0;

	  //与えられた加速度を出すために必要な電流(摩擦を除く)を求める
	  float result = vals[OMEGA_WHEEL_REF_ACC]*J*INV_Kt;

	  //モーターの回転速度が0かつ指令が0でない時、最大静止摩擦力を加える
	  if(__IS_FLOAT_ZERO(vals[OMEGA_WHEEL]) && !__IS_FLOAT_ZERO(vals[OMEGA_WHEEL_REF]))
		  result += ROT_START_CURRENT*__FLOAT_SIGN(vals[OMEGA_WHEEL_REF]);

	  //モーターの回転速度が0でない時、摩擦力を加える
	  else if(!__IS_FLOAT_ZERO(vals[OMEGA_WHEEL_REF]))
	  {
		  result += FRIC_COMP_C1*__FLOAT_SIGN(vals[OMEGA_WHEEL]);
	  }
	  return result;
	  //return 0;
}

//速度制御用変数
float goal_vel[RECEIVE_DATA_NUM] = {0},
       enc_data_smoothed[BASE_TYPE::MOTOR_NUM] = {0},
       omega_motor[BASE_TYPE::MOTOR_NUM] = {0},
       enc_data_pre[BASE_TYPE::MOTOR_NUM] = {0};
BASE_TYPE::MotorSpeed motor_goal_vel;
//BASE_TYPE::Encoder enc_data;

//電流制御用変数
float i_ref[BASE_TYPE::MOTOR_NUM] = {0},
		i_data_s[BASE_TYPE::MOTOR_NUM] = {0};
BASE_TYPE::MotorCmd ctrl_in;

//ROSSerial用の変数
void VelCmdCb(const abu2022_msgs::BaseCmd &cmd)
{
	goal_vel[0] = cmd.vx;
	goal_vel[1] = cmd.vy;
	goal_vel[2] = cmd.omega;
}
ros::NodeHandle nh;
abu2022_msgs::BaseData omni_data;
ros::Publisher datapub("omni_info", &omni_data);
//tf::TransformBroadcaster broadcaster;
//geometry_msgs::TransformStamped ts;
//char parent_frame[] = "/odom", child_frame[] = "/base_link";
ros::Subscriber<abu2022_msgs::BaseCmd> cmdsub("cmd", VelCmdCb);
//////////////////////////

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

extern "C"{

/**
  * @brief  This function configures a instance of RobotBase class and rosserial.
  * @retval None
  */
void SetParams(void){
	  //set parameters
	  BASE_TYPE::Params omniparams;
//	  MD_TYPE::Params mdparams;
//	  IMU_TYPE::Params imuparams;
//	  CURR_SENS_TYPE::Params csensparams;
	  PIDSettings vel_pid_set, curr_pid_set;

	  omniparams.enc_params.enc_handler[0] = &htim3;
	  omniparams.enc_params.enc_handler[1] = &htim4;
	  omniparams.enc_params.enc_handler[2] = &htim5;
	  omniparams.enc_params.enc_handler[3] = &htim8;
	  //omniparams.enc_params.enc_resol = 2048*4;
	  //omniparams.motor_num = MOTOR_NUM;
	  omniparams.mecha_params.dtoc = 0.3;
	  //omniparams.wheel_radius = 0.05;
	  //omniparams.dt_pid = (float)dt_pid/1000.0;
	  //omniparams.ks = 0.6; //param of LPF

	  omniparams.md_params.PHASE_Pins[0].PinNo = PHASE1_Pin;
	  omniparams.md_params.PHASE_Pins[0].Port = PHASE1_GPIO_Port;
	  omniparams.md_params.PWMH_Pins[0].PWMHandler = &htim1;
	  omniparams.md_params.PWMH_Pins[0].PWMChannel = TIM_CHANNEL_4;

	  omniparams.md_params.PHASE_Pins[1].PinNo = PHASE2_Pin;
	  omniparams.md_params.PHASE_Pins[1].Port = PHASE2_GPIO_Port;
	  omniparams.md_params.PWMH_Pins[1].PWMHandler = &htim1;
	  omniparams.md_params.PWMH_Pins[1].PWMChannel = TIM_CHANNEL_1;

	  omniparams.md_params.PHASE_Pins[2].PinNo = PHASE3_Pin;
	  omniparams.md_params.PHASE_Pins[2].Port = PHASE3_GPIO_Port;
	  omniparams.md_params.PWMH_Pins[2].PWMHandler = &htim1;
	  omniparams.md_params.PWMH_Pins[2].PWMChannel = TIM_CHANNEL_3;

	  omniparams.md_params.PHASE_Pins[3].PinNo = PHASE4_Pin;
	  omniparams.md_params.PHASE_Pins[3].Port = PHASE4_GPIO_Port;
	  omniparams.md_params.PWMH_Pins[3].PWMHandler = &htim1;
	  omniparams.md_params.PWMH_Pins[3].PWMChannel = TIM_CHANNEL_2;

	  omniparams.curr_sens_params.adc_data = new uint16_t[BASE_TYPE::MOTOR_NUM];
	  omniparams.curr_sens_params.hadc = &hadc1;

//	  omniparams.imu_params.dev_addr = 0x68<<1; //1ビット左シフトしたものを使う（超重要ポイント）
//	  omniparams.imu_params.hi2c = &hi2c1;

	  vel_pid_set.kp = VEL_PID_Kp;
	  vel_pid_set.ki = VEL_PID_Ki;
	  vel_pid_set.kd = VEL_PID_Kd;
	  vel_pid_set.ts = DT_VEL_PID;
	  //vel_pid_set.tdel = 0.015;
	  vel_pid_set.vmax = CURR_MAX;
	  vel_pid_set.vmin = -CURR_MAX;
	  vel_pid_set.mode = VEL_PID_MODE;
	  //vel_pid_set.FFFunc = FFFunc;


	  curr_pid_set.kp = CURR_PID_Kp;
	  curr_pid_set.ki = CURR_PID_Ki;
	  curr_pid_set.kd = CURR_PID_Kd;
	  curr_pid_set.ts = DT_CURRENT_PID;
	  //curr_pid_set.tdel = 0;
	  curr_pid_set.vmax = ARR_MAX;
	  curr_pid_set.vmin = -ARR_MAX;
	  curr_pid_set.mode = CURR_PID_MODE;

	  //create robot_base handler
	  omni4 = new RobotBase<BASE_TYPE>(omniparams);

	  //PIDコントローラの生成
	  vel_pid_controller = new PIDCalculator*[BASE_TYPE::MOTOR_NUM];
	  current_pid_controller = new PIDCalculator*[BASE_TYPE::MOTOR_NUM];
	  for(int i=0; i<BASE_TYPE::MOTOR_NUM; i++)
	  {
		  vel_pid_controller[i] = new PIDCalculator(vel_pid_set);
		  current_pid_controller[i] = new PIDCalculator(curr_pid_set);
	  }


	  //rosserialの初期化
	  nh.initNode();
	  nh.advertise(datapub);
	  //broadcaster.init(nh);
	  nh.subscribe(cmdsub);


	  //IMUの設定
/*	  uint8_t who_am_i;
	  HAL_StatusTypeDef mpu_config_status;
	  //x軸ジャイロをクロックソースに使用し、DLPFのBWは94Hzに設定し、ジャイロのレンジは1000deg/sに設定する
	  uint8_t config_data[3] = {0b00000001, 0b00000010, 0b00010000};

	  HAL_Delay(120); //MPUが起動するまで待つ(データシートによれば、100msくらい起動に要する)

	  mpu_config_status = HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, MPU_WHO_AM_I, 1, &who_am_i, 1, 20);
	  if(mpu_config_status == HAL_OK && who_am_i == 0x68)
	  {
		  mpu_config_status = static_cast<HAL_StatusTypeDef>(HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, MPU_PWR_MGMT_1, 1, config_data, 1, 20) +
				  	  	  	  HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, MPU_CONFIG, 1, config_data+1, 1, 20) +
							  HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, MPU_GYRO_CONFIG, 1, config_data+2, 1, 20));

		  if(mpu_config_status == HAL_OK) is_imu_ok = true;
		  else is_imu_ok = false;
	  }

	  //設定が正常に終了した場合、データ取得を開始
	  if(is_imu_ok)
	  {
		  HAL_I2C_Mem_Read_DMA(&hi2c1, MPU_ADDR, MPU_GYRO_Z, 1, imu_data, 2);
		  timeout_counter = HAL_GetTick();
	  }*/
}


/**
  * @brief  This funciton performs reading and smoothing goal velocity,
  * 		converting them to each motor's rotation speed,
  * 		and publishing odometry information to PC.
  * @retval None
  */
void MainLoop(void){
	  float goal_vel_smoothed_fixed[RECEIVE_DATA_NUM] = {0};

	  /*auto ArraySum = [](const float *goal_vels, uint16_t dim) -> float{
		  float res = 0;
		  for(int i=0; i<dim; i++) res += goal_vels[i];
		  return res;
	  };*/
	  //static BASE_TYPE::SensorData gyro_vth;
	 //目標値取得
	  nh.spinOnce();

	  //ROSに接続していない時は、停止
	  if(!nh.connected())
	  {
		  goal_vel[0] = 0;
		  goal_vel[1] = 0;
		  goal_vel[2] = 0;
	  }

	  /*IMUからのデータ取得とエラー処理*/
	  //回転速度の測定
	  //BASE_TYPE::IMUData gyro_z;

	  //if(is_imu_ok) is_imu_ok = omni4->GetIMUData(gyro_z); //I2C通信の関数実行中は、割り込みがかからないっぽい

	  //uint8_t error_flag = 0;
	  //gyro_z[0] = 0;
	  //IMUが応答していないor挙動がおかしい場合、エラーフラグを立てる
	  /*if((is_imu_ok == true) && (HAL_GetTick() - timeout_counter > IMU_TIMEOUT_TH))
	  {
		  timeout_counter = 0;
		  is_imu_ok = false;
	  }*/
		  //error_flag |= 0b00000001;
	  /*if(fabs(gyro_z[0] - gyro_z_pre)>IMU_I2C_ERROR_DETECT_TH)
	  {
		  gyro_z[0] = gyro_z_pre;
		  if(imu_malfunc_counter < IMU_MALFUNC_DETECT_CNT_TH) imu_malfunc_counter++;
		  else
		  {
			  error_flag |= 0b00000010;
			  gyro_z[0] = 0;
		  }

	  }*/
	  //エラー発生していない時は角度計算を行い、発生していれば0にする
	  //if(error_flag == 0)
	  /*if(is_imu_ok)
	  {
		  omni_theta += gyro_z*DT_MAIN; //yaw角の計算
	  }
	  else
	  {
		  gyro_z = 0;
		  omni_theta = 0;
	  }*/
	  //gyro_z_pre = gyro_z[0];

	  //台形制御指令の生成
	  for(int i=0; i<RECEIVE_DATA_NUM; i++){
		float acc = (i==OMEGA) ? ACC_ROT : ACC_VEL;

	    if(goal_vel_smoothed[i] < goal_vel[i]) goal_vel_smoothed[i] += acc*DT_MAIN;
	    else if(goal_vel_smoothed[i] > goal_vel[i]) goal_vel_smoothed[i] -= acc*DT_MAIN;
	    if((goal_vel_smoothed[i] - goal_vel[i])*(goal_vel_smoothed_pre[i] - goal_vel[i]) <= 0) goal_vel_smoothed[i] = goal_vel[i];

	    //yaw角修正FB
//	    if(i == OMEGA)
//	    {
//	    	/*if( __IS_FLOAT_ZERO(goal_vel_smoothed[OMEGA]) &&
//	    	  !(
//	    		__IS_FLOAT_ZERO(goal_vel_smoothed[VX] - goal_vel_smoothed_pre[VX]) &&
//	  	        __IS_FLOAT_ZERO(goal_vel_smoothed[VY] - goal_vel_smoothed_pre[VY])
//				)
//			)*/
//	    	if(__IS_FLOAT_ZERO(goal_vel_smoothed[OMEGA]) && (gyro_z<-OMEGA_FIX_TH || gyro_z>OMEGA_FIX_TH))
//	    	{
//	    		goal_vel_smoothed_fixed[OMEGA] = -Kp_YAW_FIX*gyro_z;
//	    	}
//	    	else
//	    	{
//	    		goal_vel_smoothed_fixed[OMEGA] = goal_vel_smoothed[OMEGA];
//	    	}
//	    }
//	    else
//	    {
	    	goal_vel_smoothed_fixed[i] = goal_vel_smoothed[i];
//	    }

	    goal_vel_smoothed_pre[i] = goal_vel_smoothed[i];
	  }
//	  //移動平均フィルタを用いて指令値を平滑
//	  for(int i=0; i<RECEIVE_DATA_NUM; i++){
//		  goal_vel_smoothed[i] = (goal_vel[i] + ArraySum(goal_vel_pre[i], N_FILTER-1))/N_FILTER;
//		  for(int j=0; j<N_FILTER-1; j++)
//		  		  goal_vel_pre[i][j] = (j==N_FILTER-2) ? goal_vel[i] : goal_vel_pre[i][j+1];
//	  }

	  /*fix robot's yaw angle*/
	  //for (int i=0; i<RECEIVE_DATA_NUM; i++) goal_vel_smoothed_fixed[i] = goal_vel_smoothed[i];
	  //for (int i=0; i<RECEIVE_DATA_NUM; i++) goal_vel_smoothed_fixed[i] = goal_vel[i];
	  //yaw角修正FB
	  //goal_vel_smoothed_fixed[VX] = goal_vel_smoothed[VX];
	  //goal_vel_smoothed_fixed[VY] = goal_vel_smoothed[VY];
	  //goal_vel_smoothed_fixed[OMEGA] = goal_vel_smoothed[OMEGA];
//	  if(__IS_FLOAT_ZERO(goal_vel_smoothed[OMEGA]) &&
//			  !__IS_FLOAT_ZERO(goal_vel_smoothed[VX]*goal_vel_smoothed[VX]+goal_vel_smoothed[VY]*goal_vel_smoothed[VY]){
//		  goal_vel_smoothed_fixed[OMEGA] = -KP_YAW_FIX*gyro_z[0];
//	  }
//	  else
//	  {
//		  goal_vel_smoothed_fixed[OMEGA] = goal_vel_smoothed[OMEGA];
//	  }

	  //convert vx, vy, vth to velocity of each motor
	  omni4->ConvToMotorVels(goal_vel_smoothed_fixed, motor_goal_vel);

	  //角速度に変換し、ついでに角速度の微分値を求める
	  for(int i=0; i<BASE_TYPE::MOTOR_NUM; i++)
	  {
		  motor_goal_vel[i] *= INV_WHEEL_RADIUS;
		  motor_goal_vel_acc[i] = (motor_goal_vel[i] - motor_goal_vel_pre[i])*INV_DT_MAIN;
		  motor_goal_vel_pre[i] = motor_goal_vel[i];
	  }

	  //エンコーダカウンタとオドメトリデータをPCに送信
//	  omni_data.x = pos_x;
//	  omni_data.y = pos_y;
//	  omni_data.theta = pos_th;
//	  float i_f = 0, i_f_sum = 0;
//	  for(int i=0; i<BASE_TYPE::MOTOR_NUM; i++)
//	  {
//		  i_f = i_data_s[i] - J_DIV_Kt*motor_goal_vel_acc[i] - b_DIV_Kt*omega_motor[i] - __FLOAT_SIGN(omega_motor[i])*Fo;
//		  i_f_sum += i_f;
//	  }
//	  omni_data.x = goal_vel_smoothed[VX];
//	  omni_data.y = goal_vel_smoothed[VY];
//	  omni_data.theta = omni_theta;
	  omni_data.delta_x = omni_delta_x;
	  omni_data.delta_y = omni_delta_y;
	  omni_data.delta_theta = omni_delta_theta;
	  omni_delta_x = 0;
	  omni_delta_y = 0;
	  omni_delta_theta = 0;
//	  omni_data.e1 = 0;
//	  omni_data.e2 = 0;
//	  omni_data.e3 = 0;
//	  omni_data.e4 = 0;
//	  //omni_data.error_flag = error_flag;
	  datapub.publish(&omni_data);
//	  ts.header.frame_id = parent_frame;
//	  ts.header.stamp = nh.now();
//	  ts.child_frame_id = child_frame;
//	  ts.transform.translation.x = omni_x;
//	  ts.transform.translation.y = omni_y;
//	  ts.transform.translation.z = 0;
//	  ts.transform.rotation.x = 0;
//	  ts.transform.rotation.y = 0;
//	  ts.transform.rotation.z = sin(0.5*omni_theta);
//	  ts.transform.rotation.w = cos(0.5*omni_theta);
//	  broadcaster.sendTransform(ts);


	  //ウォッチドッグタイマーのリセット
	  HAL_IWDG_Refresh(&hiwdg);

}
/**
  * @brief  This funciton performs PID control calculations to control rotation speed of motors.
  * @retval None
  */
void VelCtrlLoop(void)
{
	  float FF_vals[3] = {0};
	  float delta_1 = 0, delta_2 = 0, delta_3 = 0;
	  float delta_x = 0, delta_y = 0, delta_theta = 0;
	  //float omega_motor[BASE_TYPE::MOTOR_NUM] = {0};

	  BASE_TYPE::Encoder enc_data;

	  //static BASE_TYPE::MotorPWM ctrl_in;
	  //エンコーダカウンタの変化値を取得(取得後、カウンタリセット)
	  omni4->GetEncoderCounter(enc_data, true);

	  //エンコーダカウンタ値の平滑処理
	  for(int i=0; i<BASE_TYPE::MOTOR_NUM; i++)
	  {
		  enc_data_smoothed[i] = Ks_ENC*enc_data[i] + (1.0-Ks_ENC)*enc_data_pre[i];
		  enc_data_pre[i] = enc_data_smoothed[i];

		  //エンコーダカウンタの変化値から、モーターの回転角速度を計算
		  omega_motor[i] = enc_data_smoothed[i]*CONV_ENCD_TO_OMEGA;
	  }

	  //オドメトリ計算
	  delta_1 = enc_data_smoothed[0]*CONV_ENCD_TO_DISP;
	  delta_2 = enc_data_smoothed[1]*CONV_ENCD_TO_DISP;
	  delta_3 = enc_data_smoothed[2]*CONV_ENCD_TO_DISP;
	  delta_x = INV_SQRT2*(delta_1 - delta_2);
	  delta_y = INV_SQRT2*(delta_2 - delta_3);
	  delta_theta = 0.5*INV_DIST_TO_CENTER*(delta_1 + delta_3);

	  omni_delta_x += delta_x;
	  omni_delta_y += delta_y;
	  omni_delta_theta += delta_theta;

	  //速度制御用2自由度速度型PIDコントローラ(摩擦補償付き)
	  //calculate PID controller output and drive motors*/
	  for(int i=0; i<BASE_TYPE::MOTOR_NUM; i++)
	  {
		  FF_vals[OMEGA_WHEEL_REF_ACC] = motor_goal_vel_acc[i]; //速度指令の角加速度を代入
		  FF_vals[OMEGA_WHEEL_REF] = motor_goal_vel[i]; //角速度指令
		  FF_vals[OMEGA_WHEEL] = omega_motor[i]; //角速度
		  i_ref[i] = vel_pid_controller[i]->calcValue(omega_motor[i],
				  motor_goal_vel[i], FF_vals);
		  //i_ref[i] = vel_pid_controller[i]->calcValue(omega_motor[i],
		  //				  motor_goal_vel[i]);

		  motor_goal_vel_pre[i] = motor_goal_vel[i];
	  }
	  //ctrl_in = omni4->calcPIDControllerOut(enc_data, motor_goal_vel_fixed);

	  //omni4->driveMotors(ctrl_in);

	  //VelCtrlLoopの実行周期に合わせてIMUからデータを読みだす
	  /*if(imu_data_request_flag)
	  {
		  HAL_I2C_Mem_Read_DMA(&hi2c1, MPU_ADDR, MPU_GYRO_Z, 1, imu_data, 2);
		  imu_data_request_flag = false;
	  }*/
}

/**
  * @brief  This funciton performs PID control calculations to control each motor's current.
  * @retval None
  */
void CurrentCtrlLoop(void)
{
	//BASE_TYPE::MotorCmd ctrl_in;
	BASE_TYPE::CurrSensData i_data;

	//電流センサから電流値を取得
	omni4->GetCurrentSensorData(i_data);


	for(int i=0; i<BASE_TYPE::MOTOR_NUM; i++)
	{
		//値の平滑
		i_data_s[i] = Ks_CURRENT*i_data[i] + (1.0 - Ks_CURRENT)*i_data_s[i];

		//電流制御用速度型PIDコントローラ
		ctrl_in[i] = (int16_t)current_pid_controller[i]->calcValue(i_data_s[i], i_ref[i]);
	}

	//モータードライバへPWM信号を伝える
	omni4->DriveMotors(ctrl_in);
}

//受信完了時に呼ばれるコールバック関数
/*void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//データのデコード
	gyro_z = CONV_TO_DEG_PER_SEC*(int16_t)(((uint16_t)imu_data[0]<<8) | imu_data[1]);
	gyro_z += 0.050; //バイアス修正
	if(gyro_z > -0.5 && gyro_z < 0.5) gyro_z = 0; //ノイズ除去
	if(gyro_z != 0) gyro_z = gyro_z*CONV_TO_RAD; //rad/sに変換

	//タイムアウトカウンタを更新
	timeout_counter = HAL_GetTick();

	//データ受信処理開始要求フラグを立てる
	imu_data_request_flag = true;
}*/

}
