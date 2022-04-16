#ifndef IseMotorDriver_h
#define IseMotorDriver_h
#include "Arduino.h"

//足回り駆動用MD仕様

//TOP=255
#define PWM_64KHZ 0x01
#define PWM_8KHZ 0x02
#define PWM_1KHZ 0x03

#define SM_BRAKE_LOW_SIDE 0x00
#define SM_BRAKE_HIGH_SIDE 0x01
#define SM_COAST 0x02
#define LAP 0x03 //伊勢モードラでは使用不可

class IseMotorDriver {
  public:
    IseMotorDriver(uint8_t i2caddr);
    static void begin();
    bool operator << (const int &); //データ送信演算子(速度指令)
    bool operator << (const byte &); //データ送信演算子(設定値指令)
    bool operator >> (long &); //データ受信演算子
    bool operator !() const; //mdが使用可能かチェックする演算子
    static byte createSettingData(uint8_t, uint8_t); //パラメータ設定を行う関数
  private:
    uint8_t addr, SLA_W, SLA_R;
    bool start_i2c();
    bool write_addr(uint8_t);
    bool write_byte(uint8_t);
    bool read_byte(uint8_t *, char);
    void stop_i2c();
};
#endif
