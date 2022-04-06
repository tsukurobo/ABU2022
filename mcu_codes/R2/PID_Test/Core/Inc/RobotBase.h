#ifndef ROBOT_BASE_H
#define ROBOT_BASE_H

//#define RECEIVE_DATA_NUM 3

#define PI 3.141592

#define NORMAL 1
#define BUMPLESS 2
#define TWO_DOF 3
#define NORMAL_WITH_FF 3
#define BUMPLESS_WITH_FF 4


template <class BaseType/*, class MDType, class SensorType*/> class RobotBase{
private:
//	PIDSettings pidset;
//	PIDCalculator **pidc;
	//typename BaseType::MotorSpeed goal_vel; //'typename' is necessary bacause BaseType::XXX is a dependent type
	//typename BaseType::Params base_params;
	BaseType base_instance;
//	typename SensorType::Params sensparams;
//	typename MDType::Params mdparams;


public:
	RobotBase(typename BaseType::Params& pa/*, const typename SensorType::Params& pasens, const typename MDType::Params& pamd,
			const PIDSettings &pidp*/):
		/*goal_vel(), pidset(pidp), base_params(pa),*/ base_instance(pa)/*, sensparams(pasens), mdparams(pamd)*/
	{
		  //BaseType::init(baseparams);
//		  MDType::init(mdparams);


		  //initialize pid-controllers
//		  pidc = new PIDCalculator*[baseparams.motor_num];
//		  for(int i=0; i<baseparams.motor_num; i++) pidc[i] = new PIDCalculator(pidset);
	}


	//void setMotorSpeed(float (&)[RECEIVE_DATA_NUM]); //set goal speed
	//you can't return an array
	void ConvToMotorVels(const float *v_in, typename BaseType::MotorSpeed &v_out)
	{
		base_instance.ConvToMotorVels(v_in, v_out);
	}


	void GetEncoderCounter(typename BaseType::Encoder &e, bool reset_flag)
	{
		//read encoder counters
		base_instance.GetEncoderCounter(e, reset_flag);
	}


	/*typename BaseType::MotorPWM calcPIDControllerOut(const typename BaseType::Encoder& enc, const typename BaseType::MotorSpeed& mspd){ //calculate control input using PID method
		  typename BaseType::MotorPWM mpwm;
		  typename BaseType::MotorSpeed wheelspd;

		  for(int i=0; i<baseparams.motor_num; i++){
			  //convert the displacement of encoder counter to the rotation speed
			  wheelspd.v[i] = 2.0*PI*enc.e[i]/baseparams.enc_resol*baseparams.wheel_radius/baseparams.dt_pid;

			  //calculate the PID controller output
			  mpwm.p[i] = (int16_t)pidc[i]->calcValue(mspd.v[i]-wheelspd.v[i], mspd.v[i]);
		  }

		  return mpwm;
	}*/


	bool GetIMUData(typename BaseType::IMUData &data)
	{
		return base_instance.GetIMUData(data);
	}

	void GetCurrentSensorData(typename BaseType::CurrSensData &data)
	{
		base_instance.GetCurrentSensorData(data);
	}

	void DriveMotors(const typename BaseType::MotorCmd& cmd)
	{
		base_instance.DriveMotors(cmd);
	}
};


struct PIDSettings
{
  float kp = 0, ki = 0, kd = 0, ts = 0, tdel = 0, vmax = 0, vmin = 0;
  float (*FFFunc)(float *) = [](float *vals) -> float {return 0;}; //default function for FF
  uint8_t mode=NORMAL;
};


class PIDCalculator
{
private:
  PIDSettings pidset;
  //float vmax, vmin; //for anti-windup
  float FB = 0, u = 0, pre_e = 0, pre_pre_e = 0, err_sum = 0; //u is output
  float P = 0, I = 0, D = 0, D_pre = 0;
  const float inv_dt = 0;

public:
    //float kp, ki, kd, ts, tdel, a1, a2;
	PIDCalculator(PIDSettings s)
	: pidset(s), inv_dt(1.0/pidset.ts){}

	float calcValue(float curr_val, float goal_val,float *ff_vals = NULL)
	{
	  float curr_e = goal_val - curr_val;

	  switch (pidset.mode)
	  {
	    case NORMAL: //位置型PID
	      err_sum += curr_e*pidset.ts;
	      P = pidset.kp*curr_e;
	      I = pidset.ki*err_sum;
	      D = (pidset.tdel/(pidset.ts+pidset.tdel))*D + (pidset.kd/(pidset.ts+pidset.tdel))*(curr_e - pre_e); //with LPF
	      u = P + I + D;

	      //anti windup
	      if(u > pidset.vmax)
	      {
	        u = pidset.vmax;
	        err_sum -= curr_e*pidset.ts;
	      }
	      else if(u < pidset.vmin)
	      {
	        u = pidset.vmin;
	        err_sum -= curr_e*pidset.ts;
	      }

	      pre_e = curr_e;

	      return u;

	    case BUMPLESS: //速度型PID
	      float FF = pidset.FFFunc(ff_vals);

	      P = pidset.kp*(curr_e - pre_e);
	      I = pidset.ki*pidset.ts*curr_e;
	      //D = kd*(tdel/(ts+tdel))*D + (kd/(ts+tdel))*(curr_e - pre_e);
	      D = pidset.kd*(curr_e - 2.0*pre_e + pre_pre_e)*inv_dt;
	      //FB += P + I + (D-D_pre);
	      FB += P + I + D;
	      u = FB + FF;

	      if(u > pidset.vmax)
	      {
	    	  u = pidset.vmax;
	    	  //FB -= P + I + (D-D_pre);
	    	  FB -= P + I + D;
	      }
	      else if(u < pidset.vmin)
	      {
	    	  u = pidset.vmin;
	    	  //FB -= P + I + (D-D_pre);
	    	  FB -= P + I + D;
	      }

	      pre_pre_e = pre_e;
	      pre_e = curr_e;

	      //D_pre = D;

	      return u;

	/*      //2DOF PID
	      case TWO_DOF:
	      //FF
	      u = (goal_v == 0) ? 0 : ((goal_v>0) ? (a1*goal_v + a2) : (a1*goal_v - a2));

	      //FB
	      err_sum += curr_e*ts;
	      P = kp*curr_e;
	      I = ki*err_sum;
	      D = (tdel/(ts+tdel))*u + (kd/(ts+tdel))*(curr_e - pre_e); //with LPF
	      u += P + I + D;

	      //anti windup
	      if(u > vmax){
	        u = vmax;
	        err_sum -= curr_e*ts;
	      }else if(u < vmin){
	        u = vmin;
	        err_sum -= curr_e*ts;
	      }

	      pre_e = curr_e;

	      return u;*/
	  }
	}
  /*float getP();
  float getI();
  float getD();*/


};

#endif
