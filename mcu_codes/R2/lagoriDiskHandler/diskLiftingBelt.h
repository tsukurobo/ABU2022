#pragma once
#include "diskLiftingConstants.h"
#include <Arduino.h>
#include <ros.h>

extern ros::NodeHandle nh;

class Belt
{
private:
    // long target = 0;
    long initialPosition = 0; // encoderがこの値の時にifTouch==trueになるように実装
    int mode = 2;
    int manualDirection = 0;
    int autoDirection = 0;
    const int encoderPositiveDirection; // 1だと上向き、-1だと下向き
    const int pwmPositiveDirection;
    const int touchPin;
    long motionRange;
    const double convertingCoefficient;
    const int pwmUpward /* = DEFAULT_PWM */;
    const int pwmDownward /*  = DEFAULT_PWM */;

public:
    /**
     * @brief Construct a new Belt object
     *
     * @param encoderPositiveDirection
     * @param touchPin Arduinoのタッチピンが刺さってるポート番号
     * @param motionRange 可動範囲[mm]
     * @param pwmPositiveDirection
     * @param pwmUpward
     * @param pwmDownward
     * @param radius
     */
    Belt(int encoderPositiveDirection, int touchPin, long motionRange, int pwmPositiveDirection, int pwmUpward, int pwmDownward, double radius);
    uint8_t flag = 0;
    long encoder = 0;

    long convertEncoderToMM(const long &encoder) const;
    long convertMMToEncoder(const long &mm) const;

    /**
     * @brief Modeを取得
     * @details
     * @return int 0…Initialization中 1…自動昇降中 2…停止中 3…手動昇降中 4…自動上昇待機中 5…自動上昇中
     */
    int getMode() const;
    /**
     * @brief Modeを設定
     * @details
     * @param value 0…Initialization中 1…自動昇降中 2…停止中 3…手動昇降中 4…自動上昇待機中 5…自動上昇中
     */
    void setMode(const int &value);
    /**
     * @brief 移動量を取得
     *
     * @return long [パルス]
     */
    long getCurrentMovement() const;
    /**
     * @brief 移動量を[mm]で取得
     *
     * @return long [mm]
     */
    long getCurrentMovementInMM() const;
    /**
     * @brief 直ちに停止する
     *
     */
    void immediateStop();
    /**
     * @brief 動作範囲を[mm]で取得
     *
     * @return long [mm]
     */
    long getMotionRange() const;
    /**
     * @brief 初期化を開始する
     *
     */
    void startInitialization();
    /**
     * @brief 現在位置を初期位置とする
     *
     */
    void setCurrentPositionAsInitialPosition();
    /**
     * @brief マニュアル動作モードを設定する
     *
     * @param value マニュアル動作モード
     */
    void setManualDirection(const int &value);
    /**
     * @brief 自動移動時の方向を設定する
     *
     * @param direction 1…上 -1…下
     */
    void setAutoDirection(const int &direction);
    /**
     * @brief 移動範囲を再設定する
     *
     * @param range
     */
    void setMotionRange(const long &range);
    /**
     * @brief タッチセンサに触れているか
     *
     * @return true
     * @return false
     */
    bool ifTouch() const;
    /**
     * @brief タッチセンサに触れているとして、停止していいか
     *
     * @return true
     * @return false
     */
    bool ifImmediateStopPossible() const;
    /**
     * @brief 現在の高さを[mm]で取得する
     *
     * @return long 現在の高さ[mm]
     */
    virtual long getCurrentHeightInMM() const = 0;
    /**
     * @brief PWM値を返す
     *
     * @return int PWM
     */
    int pwm();
};

class Movable : public Belt
{
public:
    using Belt::Belt;
    long getCurrentHeightInMM() const override;
};

class Arm : public Belt
{
private:
    const Movable &movable;
    long targetHeight = 620;

public:
    // using Belt::Belt;
    Arm::Arm(int encoderPositiveDirection, int touchPin, long motionRange, int pwmPositiveDirection, int pwmUpward, int pwmDownward, double radius, Movable &movable);
    long getCurrentHeightInMM() const override;
    long getTargetHeight() const;
    void setTargetHeight(const long &value);
    /**
     * @brief Set the Motion Range With Disk Catcher Rotator Form object
     *
     * @param newForm (True:下 False：横)
     */
    void setMotionRangeWithDiskCatcherRotatorForm(const bool &newForm);
};

inline int Belt::getMode() const { return mode; }
inline void Belt::setMode(const int &value)
{
    if (mode != value)
    {
        mode = value;
        String info = "Mode set to " + String(value);
        nh.loginfo(info.c_str());
    }
}
inline long Belt::getCurrentMovement() const { return encoder - initialPosition; }
inline void Belt::immediateStop() { mode = 2; }
inline long Belt::getMotionRange() const { return motionRange; }
inline void Belt::startInitialization() { mode = 0; }
inline void Belt::setCurrentPositionAsInitialPosition() { initialPosition = encoder; }
inline void Belt::setManualDirection(const int &value)
{
    manualDirection = value;
    if (mode == 2 || mode == 5)
    {
        setMode(3);
    }
}
inline void Belt::setAutoDirection(const int &direction)
{
    autoDirection = direction;
}

inline long Arm::getTargetHeight() const { return targetHeight; }
inline void Arm::setTargetHeight(const long &value) { targetHeight = value; }
inline void Belt::setMotionRange(const long &range)
{
    motionRange = range;
}
inline void Arm::setMotionRangeWithDiskCatcherRotatorForm(const bool &newForm)
{
    setMotionRange(ARM_MOTION_RANGE + (newForm ? 0 : ARM_ADDITIONAL_RANGE_WHEN_CATCHER_HORIZONTAL));
}