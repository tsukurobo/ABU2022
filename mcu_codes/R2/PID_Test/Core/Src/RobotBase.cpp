/*#include "RobotBase.h"

template <class BaseType, class SensorType, class MDType> RobotBase<BaseType, SensorType, MDType>::RobotBase(const typename BaseType::Params& pa,
		const typename SensorType::Params& pasens, const typename MDType::Params& pamd) :
goal_vel(), baseparams(pa), sensparams(pasens), mdparams(pamd), convmat(pa){
  BaseType::init(baseparams);
}

template <class BaseType, class SensorType, class MDType> typename BaseType::MotorSpeed RobotBase<BaseType, SensorType, MDType>::convVXYW2MotorVels(float (&v)[RECIEVE_DATA_NUM]){
  typename BaseType::MotorSpeed mvels;
  for(int i=0; i<convmat.row; i++){
	  for(int j=0; j<convmat.col; j++){
		  mvels.v[i] += convmat.data[convmat.col*i+j]*v[j];
	  }
  }
  return mvels;
}

template <class BaseType, class SensorType, class MDType> typename BaseType::Encoder RobotBase<BaseType, SensorType, MDType>::getEncoder(){
  return BaseType::getEncoder(baseparams);
}

template <class BaseType, class SensorType, class MDType> typename BaseType::MotorPWM RobotBase<BaseType, SensorType, MDType>::calcPIDControllerOut(const typename BaseType::Encoder& enc_delta,
		const typename BaseType::MotorSpeed& mspds){
  typename BaseType::MotorPWM mpwm;
  return mpwm;
}

template <class BaseType, class SensorType, class MDType> typename SensorType::SensorData RobotBase<BaseType, SensorType, MDType>::getImuData(){
  typename SensorType::SensorData retval;
  return retval;
}

template <class BaseType, class SensorType, class MDType> void RobotBase<BaseType, SensorType, MDType>::driveMotors(const typename BaseType::MotorPWM&){

}*/
