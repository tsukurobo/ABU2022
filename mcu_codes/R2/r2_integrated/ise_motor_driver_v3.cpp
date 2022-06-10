#include "ise_motor_driver_v3.h"

#define START 0x08
#define MT_SLA_ACK 0x18
#define MT_DATA_ACK 0x28
#define MR_SLAR_ACK 0x40
#define MR_DATA_ACK 0x50
#define MR_DATA_NACK 0x58
#define READ_BIT0_TO_6 0x7F
#define READ_BIT0_TO_7 0xFF
#define MASK_BIT0_TO_2 0xF8

#define WAIT_T_BET_TRANS 20
#define WAIT_T_BET_DATA 5

IseMotorDriver::IseMotorDriver(uint8_t i2caddr){
  // @param uint8_t i2caddr: i2c addr of motor driver
  //initializer
  this->addr = i2caddr;
  SLA_W = i2caddr<<1;
  SLA_R = i2caddr<<1 | 1;
}

void IseMotorDriver::begin(){
  digitalWrite(SDA, HIGH); //SDA, SCLピンをプルアップ
  digitalWrite(SCL, LOW);
  TWBR |= 0x0c; //通信速度400kbps
}

/*実行には約110μs(@400kbps)かかる
/*注意:MDが接続されていない/応答がない場合、whileから抜け出せなくなる
 *戻り値の値は、MDがちゃんと認識されたうえで、正常に通信が完了したかどうか 
 *まずは!演算子でMDが接続されているかチェックすること
 */
bool IseMotorDriver::operator << (const int &data){
  /*13bit signed int: -4096から4096までの値が送れる(範囲外の値が送られたときの動作は未定義) 
   *{data>=0, data<0}のとき、{data, -data}の最下位から12bitを5bitと7bitに分割し、
   * 上位5bitを最初に送信し、下位7bitを次に送信する
   * また、各byteの最上位bitは順序を表し、1番目のbyteの2番目のbitは送信するデータの種類を表す
   * (0→速度指令 1→パラメータ)
   */
  uint8_t upper, lower;
  
  if(data>=0){ //正の値を送る場合
    upper = data>>7; //12bitのうち上位5bit分
    lower = (data&READ_BIT0_TO_6) | (1<<7); //下位7bit分
  }else if(data<0){//負の値を送る場合
    unsigned int data_abs = -data; //dataの絶対値を格納
    upper = (uint8_t)(data_abs>>7) | (1<<5);
    lower = (data_abs&READ_BIT0_TO_6) | (1<<7);
  }
  
  //通信開始
  if(!start_i2c()){ //正常に通信開始が行われなかった場合、異常処理へ
    stop_i2c(); //通信終了
    return false;
  }
  //マスター送信モードに設定
  if(!write_addr(SLA_W)){ //正常にアドレスが送れなかった場合、異常処理へ
    stop_i2c();
    return false;
  }
  //上位8ビット送信
  if(!write_byte(upper)){ //正常にデータが送れなかった場合、異常処理へ
    stop_i2c();
    return false;
  }
  delayMicroseconds(WAIT_T_BET_DATA); //少し待つ
  //下位8ビット送信
  if(!write_byte(lower)){ //正常にデータが送れなかった場合、異常処理へ
    stop_i2c();
    return false;
  }

  stop_i2c();
  
  return true;

}

// 実行には約200μsかかる(@400kbps)
bool IseMotorDriver::operator >> (long &enc){
  unsigned long buf = 0;
  uint8_t buf_s;
  
  //通信開始
  if(!start_i2c()){ //正常に通信開始が行われなかった場合、異常処理へ
    stop_i2c(); //通信終了
    return false;
  }
  //マスター受信モードに設定
  if(!write_addr(SLA_R)){ //正常にアドレスが送れなかった場合、異常処理へ
    stop_i2c();
    return false;
  }
  for(int i=0; i<3; i++){
    //上位24ビットを受信
    if(!read_byte(&buf_s, 'a')){//正常にデータ受信が行われなかった場合、異常処理へ
      stop_i2c();
      return false;
    }
    buf |= (unsigned long)buf_s<<(8*(3-i));//データをバッファに格納
  }
  //下位8ビット受信
  if(!read_byte(&buf_s, 'n')){//正常にデータ受信が行われなかった場合、異常処理へ
    stop_i2c();
    return false;
  }
  buf |= (unsigned long)buf_s;//データをバッファに格納
  
  stop_i2c();

  enc = (long)buf;
  
  return true;
}

//設定データを送る
bool IseMotorDriver::operator << (const byte &config_d){
  //通信開始
  if(!start_i2c()){ //正常に通信開始が行われなかった場合、異常処理へ
    stop_i2c(); //通信終了
    return false;
  }
  //マスター送信モードに設定
  if(!write_addr(SLA_W)){ //正常にアドレスが送れなかった場合、異常処理へ
    stop_i2c();
    return false;
  }
  //8ビット送信
  if(!write_byte(config_d)){ //正常にデータが送れなかった場合、異常処理へ
    stop_i2c();
    return false;
  }
  
  stop_i2c();
  
  return true;
}

//実行には60us程度かかる
bool IseMotorDriver::operator ! () const{ //使用可能→false、使用不可→true
  const long timeout = 1000; //タイムアウトとなるまでの時間。単位:us
  long time_pre;
  bool timeoutflag = false;

  //通信開始
  if(!start_i2c()){ //正常に通信開始が行われなかった場合、異常処理へ
    stop_i2c(); //通信終了
    return false;
  }

  time_pre = micros();
  TWDR = SLA_W; //アドレスをデータレジスタに書き込み
  TWCR = (1<<TWINT | (1<<TWEN)); //アドレス送信
  while(!(TWCR & (1<<TWINT))){ //送信終了まで待つ
     if(micros() - time_pre > timeout){
      timeoutflag = true;
      break;
     }
  }
  if(timeoutflag == true){ //タイムアウトになった場合
    stop_i2c(); //通信終了
    return true;
  }
  if((TWSR & MASK_BIT0_TO_2) != MT_SLA_ACK){ //ステータスレジスタを確認。ACKが返ってこなかった場合、異常処理へ
    stop_i2c(); //通信終了
    return true;
  }
  
  stop_i2c(); //通信終了
  return false;

}

byte IseMotorDriver::createSettingData(uint8_t pwm_mode, uint8_t a3921_mode){
  byte settings;
  /*
   * MSBを1番目として2番目のbitに1、3~5番目のbitにpwm_mode、6~8番目のbitにa3921_modeを代入
   */
  settings = (pwm_mode<<3) | a3921_mode | (1<<6);
  return settings;
}

bool IseMotorDriver::start_i2c(){
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); //開始条件送信
  while(!(TWCR & (1<<TWINT))); //送信終了まで待つ
  
  if((TWSR & MASK_BIT0_TO_2) != START) return false;//ステータスレジスタを確認。正常に送信が行われなかった場合、異常処理へ

  return true;
}

bool IseMotorDriver::write_addr(uint8_t sla){
  TWDR = sla; //アドレスをデータレジスタに書き込み
  TWCR = (1<<TWINT | (1<<TWEN)); //アドレス送信
  while(!(TWCR & (1<<TWINT))); //送信終了まで待つ
  
  if((TWSR & MASK_BIT0_TO_2) != MT_SLA_ACK && (TWSR & MASK_BIT0_TO_2) != MR_SLAR_ACK) return false; //ステータスレジスタを確認。ACKが返ってこなかった場合、異常処理へ

  return true;
}

bool IseMotorDriver::write_byte(uint8_t data){
  TWDR = data; //データをデータレジスタに書き込む
  TWCR = (1<<TWINT | (1<<TWEN)); //データ送信
  while(!(TWCR & (1<<TWINT))); //送信終了まで待つ
  if((TWSR & MASK_BIT0_TO_2) != MT_DATA_ACK) return false;//ステータスレジスタを確認。ACKが返ってこなかった場合、異常処理へ

  return true;
}
/*二つ目の引数で、受信後にACKを返すかNACKを返すか設定する
 * mode=='a'でACK、それ以外の文字列でNACKを返す
 */
bool IseMotorDriver::read_byte(uint8_t *buf, char mode){
  TWCR = (mode=='a') ? (1<<TWINT) | (1<<TWEN) | (1<<TWEA) : (1<<TWINT) | (1<<TWEN); //データを受信する
  while(!(TWCR & (1<<TWINT))); //受信終了まで待つ
  /*↓ステータスレジスタを確認。"modeが'a'でACK送信
   * もしくはmodeが'a'以外でNACK送信"以外の場合は異常処理へ
   */
  if((mode=='a' && (TWSR & MASK_BIT0_TO_2)==MR_DATA_ACK) || 
  (mode!='a' && (TWSR & MASK_BIT0_TO_2)==MR_DATA_NACK)){
    *buf = TWDR;
    return true;  
  }
  
  return false;
}

void IseMotorDriver::stop_i2c(){
  TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN); //終了条件送信
  //while(!(TWCR & (1<<TWINT))); //送信終了まで待つ
  delayMicroseconds(WAIT_T_BET_TRANS); //少し待つ
}
