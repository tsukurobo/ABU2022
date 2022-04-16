#ifndef OMNI4_H
#define OMNI4_H

#include "stm32f4xx_hal.h"

//#define MOTOR_NUM 4
#define RECEIVE_DATA_NUM 3
#define SQRT2 1.41421356
#define INV_SQRT2 0.7071067812
#define CONV_TO_RAD 0.0174533 //pi/180
#define INV_ADC_RESOL 0.00024420024 // = 1/4095
/*#define MPU_WHO_AM_I 0x75 //MPU6050のWHO_AM_Iレジスタのアドレス（レジスタに0x68が格納されていればok）
#define MPU_PWR_MGMT_1 0x6B //MPU6050のpower management 1レジスタのアドレス
#define MPU_CONFIG 0x1A //MPU6050のconfigurationレジスタのアドレス
#define MPU_GYRO_CONFIG 0x1B //MPU6050のgyroscope configurationレジスタのアドレス
#define MPU_GYRO_Z 0x47 //MPU6050のGYRO_ZOUT_Hのアドレス
#define MPU_GYRO_RANGE 1000.0 //単位は°/s*/
//#define INT16_MAX_VALUE 32767.0
//#define MAX_PWM_DUTY 3999


class Omni4
{
public:
	const static int MOTOR_NUM = 4; //const静的メンバ変数は、宣言と同時に初期化しましょう

private:
//public:
	template <typename T, int arrayn> class DataArray
	{
		T data[arrayn];

	public:
		T &operator[](int x){return data[x];}
		T operator[](int x)const{return data[x];}
	};

	template <int arrayn> class DataArray<float, arrayn>
	{
		float data[arrayn] = {0};

	public:
		float &operator[](int x){return data[x];}
		float operator[](int x)const{return data[x];}
	};

	template <int arrayn> class DataArray<int, arrayn>
	{
		int data[arrayn] = {0};

	public:
		int &operator[](int x){return data[x];}
		int operator[](int x)const{return data[x];}
	};

	struct MechaParams
	{
		float dtoc = 0;
	};

	struct EncParams
	{
		//uint16_t enc_resol = 0;
		TIM_HandleTypeDef *enc_handler[MOTOR_NUM];
	};

	struct GPIOPins{
		GPIO_TypeDef *Port;
		uint16_t PinNo;
	};


	struct PWMPins{
		TIM_HandleTypeDef *PWMHandler;
		uint32_t PWMChannel;
	};


	struct MDParams{
		GPIOPins PHASE_Pins[MOTOR_NUM];
		PWMPins PWMH_Pins[MOTOR_NUM];
	};

//	struct IMUParams{
//		I2C_HandleTypeDef *hi2c;
//		uint16_t dev_addr;
//	};

	struct CurrSensParams
	{
		uint16_t *adc_data;
		ADC_HandleTypeDef *hadc;
	};

public:

	/*struct MotorSpeed{
		float v[MOTOR_NUM]={0};
	};


	struct Encoder{
		float e[MOTOR_NUM]={0};
	};*/
	using MotorSpeed = DataArray<float, MOTOR_NUM>;
	using MotorCmd = DataArray<int, MOTOR_NUM>;
	using Encoder = DataArray<int, MOTOR_NUM>;
	using IMUData = DataArray<float, 1>; //gzのみ
	using CurrSensData = DataArray<float, MOTOR_NUM>;

	struct Params
	{
		MechaParams mecha_params;
		EncParams enc_params;
		MDParams md_params;
		//IMUParams imu_params;
		CurrSensParams curr_sens_params;
	};


	/*struct MotorPWM{
		int16_t p[MOTOR_NUM] = {0};
	};*/

private:
	Params pa;
	//bool is_mpu_ready = false;
	//const float CONV_TO_DEG_PER_SEC;

public:
	//コンストラクタ
	Omni4(const Params &p)
	: pa(p)/*, CONV_TO_DEG_PER_SEC(MPU_GYRO_RANGE/INT16_MAX)*/
	{
		for(int i=0; i<MOTOR_NUM; i++)
		{
			//start encoder
			HAL_TIM_Encoder_Start(pa.enc_params.enc_handler[i], TIM_CHANNEL_ALL);

			//start pwm
			__HAL_TIM_SET_COMPARE(pa.md_params.PWMH_Pins[i].PWMHandler, pa.md_params.PWMH_Pins[i].PWMChannel, 0);
			HAL_TIM_PWM_Start(pa.md_params.PWMH_Pins[i].PWMHandler, pa.md_params.PWMH_Pins[i].PWMChannel);
		}

		//adc_data = new uint16_t[MOTOR_NUM];
		HAL_ADC_Start_DMA(pa.curr_sens_params.hadc, (uint32_t*)pa.curr_sens_params.adc_data, (uint32_t)MOTOR_NUM);


		/*uint8_t who_am_i;
		HAL_StatusTypeDef mpu_config_status;

		//x軸ジャイロをクロックソースに使用し、DLPFのBWは94Hzに設定し、ジャイロのレンジは1000deg/sに設定する
		uint8_t config_data[3] = {0b00000001, 0b00000010, 0b00010000};

		HAL_Delay(200); //MPUが起動するまで待つ(データシートによれば、100msくらい起動に要する)

		mpu_config_status = HAL_I2C_Mem_Read(pa.imu_params.hi2c, pa.imu_params.dev_addr, MPU_WHO_AM_I, 1, &who_am_i, 1, 100);
		if(mpu_config_status == HAL_OK && who_am_i == 0x68)
		{
			mpu_config_status = static_cast<HAL_StatusTypeDef>(HAL_I2C_Mem_Write(pa.imu_params.hi2c, pa.imu_params.dev_addr, MPU_PWR_MGMT_1, 1, config_data, 1, 100) +
								HAL_I2C_Mem_Write(pa.imu_params.hi2c, pa.imu_params.dev_addr, MPU_CONFIG, 1, config_data+1, 1, 100) +
								HAL_I2C_Mem_Write(pa.imu_params.hi2c, pa.imu_params.dev_addr, MPU_GYRO_CONFIG, 1, config_data+2, 1, 100));

			if(mpu_config_status == HAL_OK) is_mpu_ready = true;
			else is_mpu_ready = false;
		}*/


	}

	//エンコーダの値を読み取って返すだけ
	//static MotorSpeed calcMotorSpeed(float (&)[RECEIVE_DATA_NUM], const Params&);
	void GetEncoderCounter(Encoder &e, bool reset_flag){
		//Encoder enc;
		//uint16_t enc_buf[MOTOR_NUM]={0};

		for(int i=0; i<MOTOR_NUM; i++)
		{
			//enc_buf[i] = __HAL_TIM_GET_COUNTER(pa.enc_handler[i]); //get the value of the counter
			e[i] = (int16_t)__HAL_TIM_GET_COUNTER(pa.enc_params.enc_handler[i]);
			if(reset_flag == true) __HAL_TIM_SetCounter(pa.enc_params.enc_handler[i], 0); //reset the value

			/*if(enc_buf[0] > INT16_MAX_VALUE) enc.e[0] = (int16_t)enc_buf[0];
			else enc.e[0] = (int16_t)enc_buf[0];*/
			//enc.e[i] = (float)((int16_t)enc_buf[i])*pa.ks + enc.e[i]*(1.0-pa.ks);
		}

		//return enc;
	}


	void ConvToMotorVels(const float *v_in, MotorSpeed &v_out)
	{
		//MotorSpeed spd;
		v_out[0] = 0.5*SQRT2*v_in[0] +  0.5*SQRT2*v_in[1] + pa.mecha_params.dtoc*v_in[2];
		v_out[1] = -0.5*SQRT2*v_in[0] +  0.5*SQRT2*v_in[1] + pa.mecha_params.dtoc*v_in[2];
		v_out[2] = -0.5*SQRT2*v_in[0] -  0.5*SQRT2*v_in[1] + pa.mecha_params.dtoc*v_in[2];
		v_out[3] = 0.5*SQRT2*v_in[0] -  0.5*SQRT2*v_in[1] + pa.mecha_params.dtoc*v_in[2];

		//return spd;
	}


//	static void init(const Params& pa)
//	{
//		for(int i=0; i<MOTOR_NUM; i++)
//		{
//			//start encoder
//			HAL_TIM_Encoder_Start(pa.enc_params.enc_handler[i], TIM_CHANNEL_ALL);
//
//			//start pwm
//			__HAL_TIM_SET_COMPARE(pa.md_params.PWMH_Pins[i].PWMHandler, pa.md_params.PWMH_Pins[i].PWMChannel, 0);
//			HAL_TIM_PWM_Start(pa.md_params.PWMH_Pins[i].PWMHandler, pa.md_params.PWMH_Pins[i].PWMChannel);
//		}
//
//		//adc_data = new uint16_t[MOTOR_NUM];
//		HAL_ADC_Start_DMA(pa.curr_sens_params.hadc, (uint32_t*)pa.curr_sens_params.adc_data, (uint32_t)MOTOR_NUM);
//	}

	void DriveMotors(MotorCmd cmd)
	{
		//SRとPWMLはVddに繋いでおく
		for(int i=0; i<MOTOR_NUM; i++)
		{
			//slow decay with low side recirculation
//			if(i == 3)
//			{
//				if(cmd[i]>=0) HAL_GPIO_WritePin(pa.md_params.PHASE_Pins[i].Port, pa.md_params.PHASE_Pins[i].PinNo, GPIO_PIN_RESET);
//				else HAL_GPIO_WritePin(pa.md_params.PHASE_Pins[i].Port, pa.md_params.PHASE_Pins[i].PinNo, GPIO_PIN_SET);
//			}
//			else
//			{
				if(cmd[i]>=0) HAL_GPIO_WritePin(pa.md_params.PHASE_Pins[i].Port, pa.md_params.PHASE_Pins[i].PinNo, GPIO_PIN_SET);
				else HAL_GPIO_WritePin(pa.md_params.PHASE_Pins[i].Port, pa.md_params.PHASE_Pins[i].PinNo, GPIO_PIN_RESET);
//			}

			__HAL_TIM_SET_COMPARE(pa.md_params.PWMH_Pins[i].PWMHandler, pa.md_params.PWMH_Pins[i].PWMChannel, cmd[i]>=0 ? cmd[i] : -cmd[i]);
		}
	}

	bool GetIMUData(IMUData &data)
	{
//		uint8_t imu_data[2]={0};
//
//		//start i2c communication
//		HAL_StatusTypeDef s = HAL_I2C_Mem_Read(pa.imu_params.hi2c, pa.imu_params.dev_addr,MPU_GYRO_Z, 1, imu_data, 2, 2); //2msでtimeout
//
//
//		if(is_mpu_ready == true && s == HAL_OK)
//		{
//			data[0] = CONV_TO_DEG_PER_SEC*(int16_t)(((uint16_t)imu_data[0]<<8) | imu_data[1]);
//			data[0] = data[0] + 0.050; //バイアス修正
//			if(data[0] > -0.17 && data[0] < 0.17) data[0] = 0; //ノイズ除去
//			if(data[0] != 0) data[0] = data[0]*CONV_TO_RAD; //rad/sに変換
//
//			return true;
//		}
//		else
//		{
//			data[0] = 0;
//			return false;
//		}
		return true;
	}

	//CubeMX上でのrank1~4が（多分）data[0]~data[3]に対応するんだと思う
	void GetCurrentSensorData(CurrSensData &data)
	{
		float current_buf;
		for(int i=0; i<MOTOR_NUM; i++)
		{
			if(i == 2) current_buf = 41.666667*(3.3*(float)pa.curr_sens_params.adc_data[i]*INV_ADC_RESOL - 1.462);
			else current_buf = -41.666667*(3.3*(float)pa.curr_sens_params.adc_data[i]*INV_ADC_RESOL - 1.462);

			if(current_buf < 1.0 && current_buf > -1.0) current_buf = 0;
			data[i] = current_buf;
		}
	}

};

//const int Omni4::MOTOR_NUM = 4;

/*class Gyro{
public:
	struct Params{

	};


	struct SensorData{
		float v_yaw = 0;
		uint8_t size = 1;
	};
};



class SimpleMotorDriver{
public:
	struct GPIOPins{
		GPIO_TypeDef *Port;
		uint16_t PinNo;
	};


	struct PWMPins{
		TIM_HandleTypeDef *PWMHandler;
		uint32_t PWMChannel;
	};


	struct Params{
		GPIOPins PHASE_Pins[MOTOR_NUM];
		PWMPins PWMH_Pins[MOTOR_NUM];
	};


	static void drive( const Params& pa, const Omni4::MotorPWM& pwm_){
		//SRとPWMLはVddに繋いでおく
		for(int i=0; i<MOTOR_NUM; i++){
			//slow decay with low side recirculation
			if(pwm_.p[i]>=0) HAL_GPIO_WritePin(pa.PHASE_Pins[i].Port, pa.PHASE_Pins[i].PinNo, GPIO_PIN_SET);
			else HAL_GPIO_WritePin(pa.PHASE_Pins[i].Port, pa.PHASE_Pins[i].PinNo, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(pa.PWMH_Pins[i].PWMHandler, pa.PWMH_Pins[i].PWMChannel, pwm_.p[i]>=0 ? pwm_.p[i] : -pwm_.p[i]);
		}
	}


	static void init(const Params &pa){
		for(int i=0; i<MOTOR_NUM; i++){
			__HAL_TIM_SET_COMPARE(pa.PWMH_Pins[i].PWMHandler, pa.PWMH_Pins[i].PWMChannel, 0);
			HAL_TIM_PWM_Start(pa.PWMH_Pins[i].PWMHandler, pa.PWMH_Pins[i].PWMChannel);
		}
	}
};*/

#endif
