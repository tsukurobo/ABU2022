#ifndef ACTUATOR_H
#define ACTUATOR_H


#include "ise_motor_driver_v3.h"
#include <Servo.h>
//#include <Arduino.h>

//ヘッダに全てを書いていくのは良くない(が、ここではやる)

namespace CustomDataStrageStructure
{
    
    //使う機能だけ詰めたList型
    template<typename T> class List
    {
    private:
        struct ListElement
        {
            ListElement *next;
            T data;
            
            ListElement()
            : next(NULL)
            {}
        };
        
        ListElement *begin_elem_ptr_, *current_end_elem_ptr_, *end_elem_ptr_mem_allocated_;
    
    public:
        class Iterator
        {
        private:
            ListElement *elemp_;
            
        public:
            Iterator(ListElement *p)
            : elemp_(p)
            {}
            
            //後置インクリメント演算子
            void operator ++(int n)
            {
                elemp_ = elemp_->next;
            }

            //アクセス演算子
            //const指定により、この関数はメンバ変数を変更できない
            T operator *() const 
            {
                return elemp_->data;
            }
            
            //不等価演算子
            bool operator !=(const Iterator &obj) const
            {
                return (elemp_ != obj.elemp_);
            }
        };

        //コンストラクタ
        List()
        {
            begin_elem_ptr_ = new ListElement();
            current_end_elem_ptr_ = begin_elem_ptr_;
            end_elem_ptr_mem_allocated_ = begin_elem_ptr_;
        }

        //デストラクタ
        ~List()
        {
            ListElement *delete_elem_ptr = begin_elem_ptr_,
            *delete_elem_ptr_next = begin_elem_ptr_->next;
            
            while(delete_elem_ptr != end_elem_ptr_mem_allocated_)
            {
                delete delete_elem_ptr;
                delete_elem_ptr = delete_elem_ptr_next;
                delete_elem_ptr_next = delete_elem_ptr->next;
            }
            delete delete_elem_ptr;
            
        }
        
        Iterator begin()
        {
            Iterator itr(begin_elem_ptr_->next);
            return itr;
        }

        Iterator end()
        {
            Iterator itr(current_end_elem_ptr_->next);
            return itr;
        }
        
        void append(T data)
        {
            if(current_end_elem_ptr_->next == NULL)
            {
                current_end_elem_ptr_->next = new ListElement();
                end_elem_ptr_mem_allocated_ = current_end_elem_ptr_->next;
            }
            current_end_elem_ptr_ = current_end_elem_ptr_->next;
            current_end_elem_ptr_->data = data;
        }
        
        void clearAll()
        {
            current_end_elem_ptr_ = begin_elem_ptr_; //末端要素を指すポインタの位置を最初に戻すだけ
        }
    };
}



namespace PIDController {
    struct PIDSettings{
        float kp, ki, kd, ts, tdel, vmax, vmin;
    };
    
    
    
    class PIDCalculator
    {
    private:
        float vmax, vmin;               //アンチワインドアップ発動の閾値(PWMのduty比の最大・最小値)
        float u, pre_e, err_sum;        //uは出力
        float P, I, D, D_pre;           //各項の値
        float kp, ki, kd, ts, tdel;     //計算に使うゲインとか
        float C1, C2;                   //計算量減少を図るための定数格納用
    
    public:
        PIDCalculator(const PIDSettings &s)
        : kp(s.kp), ki(s.ki), kd(s.kd), ts(s.ts), tdel(s.tdel), 
        vmax(s.vmax), vmin(s.vmin), u(0), pre_e(0), err_sum(0),D_pre(0),
        C1(tdel/(ts+tdel)), C2(kd/(ts+tdel))
        {}
        
        //誤差から出力を計算
        float calcValue(float curr_e)
        {
            err_sum += curr_e*ts;
            P = kp*curr_e;
            I = ki*err_sum;
            D = C1*D + C2*(curr_e - pre_e);     //不完全微分
            u = P + I + D;
            
            //アンチワインドアップ
            if(u > vmax){
              u = vmax;
              err_sum -= curr_e*ts;
            }else if(u < vmin){
              u = vmin;
              err_sum -= curr_e*ts;
            }
            
            pre_e = curr_e;
            
            return u;
        }
    };
}



namespace Actuator
{
    //基底クラス
    class Actuator
    {
    protected:
        const bool USE_TASK_F_;
    
    public:
        virtual long executeTask()
        {
            return 0;
        }
        
        virtual void setValue(long x){}

        virtual void reset(){}
    
        bool useTaskFunction()
        {
            return USE_TASK_F_;
        }
        
        Actuator(bool use_task)
        : USE_TASK_F_(use_task)
        {}
    };



    //モータークラス(Actuatorクラスを継承)
    class DCMotor : public Actuator
    {
    protected:
        IseMotorDriver *md_;
    
    public:
        //コンストラクタ（引数はMDのI2Cアドレス)
        DCMotor(uint8_t addr, bool use_task = false)
        : Actuator(use_task)
        {
            IseMotorDriver::begin();
            md_ = new IseMotorDriver(addr);
            if(!(*md_) == false) *md_ << 0; //最初は停止
        }
        
        //MDにCCR値を送る
        void setValue(long v)
        {
            if(!(*md_) == false) *md_ << static_cast<int>(v);
        }

        //リセット関数
        void reset()
        {
            if(!(*md_) == false) *md_ << 0;
        }
    };

  
    
    //速度制御対応型モータークラス(Motorクラスを継承)
    class DCMotorWithVelPID : public DCMotor
    {
    protected:
        const float CONV_DENC_TO_ROT_, ACC_, TS_, K_ENC_;
        
        PIDController::PIDCalculator *pidc_;
        long goal_rot_vel_ = 0, prev_enc_ = 0;
        float denc_s = 0, goal_rot_vel_s_ = 0, goal_rot_vel_s_pre_ = 0;
        
    public:
        /*
         *コンストラクタ(引数は、MDのI2Cアドレス、PIDパラメータ、モーターの加速度、
         *エンコーダ分解能、エンコーダカウンタ平滑LPF用定数)
         */
        DCMotorWithVelPID(uint8_t addr, const PIDController::PIDSettings &ps,
        int enc_resol, float acc, float k_enc = 0.4)
        
        : DCMotor(addr, true), CONV_DENC_TO_ROT_(2.0*3.141592/enc_resol/ps.ts),
          ACC_(acc), TS_(ps.ts), K_ENC_(k_enc)
        {
            pidc_ = new PIDController::PIDCalculator(ps);
            if(!(*md_) == false) *md_ >> prev_enc_;
        }

        //PID制御実行
        long executeTask()
        {
            long retval = 0;

            //台形制御指令を計算
            if(goal_rot_vel_s_pre_ < static_cast<float>(goal_rot_vel_))
                goal_rot_vel_s_ = goal_rot_vel_s_pre_ + ACC_*TS_;
            else if(goal_rot_vel_s_pre_ > static_cast<float>(goal_rot_vel_))
                goal_rot_vel_s_ = goal_rot_vel_s_pre_ - ACC_*TS_;

            if((goal_rot_vel_s_ - static_cast<float>(goal_rot_vel_))
                *(goal_rot_vel_s_pre_ - static_cast<float>(goal_rot_vel_)) <= 0)
            {
                goal_rot_vel_s_ = static_cast<float>(goal_rot_vel_);
            }
            goal_rot_vel_s_pre_ = goal_rot_vel_s_;
            
            if(!(*md_) == false)
            {   
                long curr_enc = 0;
                
                *md_ >> curr_enc;    //エンコーダ値取得
                denc_s = K_ENC_*(curr_enc - prev_enc_) + denc_s*(1.0-K_ENC_);    //カウンタ変化分を平滑
                float curr_rot_vel = CONV_DENC_TO_ROT_*denc_s;      //rad/sに変換
                float pidc_out = pidc_->calcValue(static_cast<float>(goal_rot_vel_s_) - curr_rot_vel);     //PID制御の計算(制御出力を求める)
                *md_ << static_cast<int>(pidc_out);      //MDに送る 
                
                prev_enc_ = curr_enc;
                retval = static_cast<long>(curr_rot_vel*10.0); //単位を[x10^(-1) rad/s]とする
                //retval = static_cast<long>(pidc_out);
            }

            return retval;
        }

        //目標値[rad/s]をセット
        void setValue(long v)
        {
            goal_rot_vel_ = v;
        }

        //リセット関数
        void reset()
        {
            if(!(*md_) == false) *md_ << 0;
        }
    };



    //サーボ制御クラス(Actuatorクラスを継承)
    class ServoMotor : public Actuator
    {
    protected:
        Servo servo_;
        int def_angle_;

    public:
        //コンストラクタ
        ServoMotor(int servo_pin, int default_angle)
        : Actuator(false), def_angle_(default_angle)
        {
            servo_.attach(servo_pin);
            
            if(def_angle_ < 0) def_angle_ = 0;
            else if(def_angle_ > 180) def_angle_ = 180;
            servo_.write(def_angle_);
        }
        
        //サーボを指定した角度に動かす
        void setValue(long v)
        {
            if(v < 0) v = 0;
            else if(v > 180) v = 180;
            servo_.write(static_cast<int>(v)); 
        }

        void reset()
        {
            servo_.write(def_angle_);
        }
    };



    //コンテナ
    class ActuatorContainer
    {
    private:
        CustomDataStrageStructure::List<Actuator *> actuators_;               //ポインタ格納用配列
        CustomDataStrageStructure::List<Actuator *> actuators_with_task_  ;   //executeTask()を実行するactuator
        //テンプレートパラメータはとりあえずlongにします
        CustomDataStrageStructure::List<long> return_data;     //executeTask()を実行した後の戻り値格納用
        void (*sender)(const CustomDataStrageStructure::List<long> &);
        long time_pre;
        const unsigned int loop_dt_;    //unit: ms

    public:
    
        //コンストラクタ(引数は、update()の実行周期、データ送信に使う関数のポインタ
        ActuatorContainer(unsigned int loop_dt, void (*sendf)(const CustomDataStrageStructure::List<long> &))
        : loop_dt_(loop_dt), time_pre(millis()), sender(sendf)
        {
        }
        
        //actuatorをactuators_に追加
        void append(Actuator *pact)
        {
            actuators_.append(pact);
            if(pact->useTaskFunction()) actuators_with_task_.append(pact);
            //actuator_num_++;
        }

        //loop()内で実行される関数
        void update()
        {
            long time_now = millis();

            if(time_now - time_pre > loop_dt_)
            {
                return_data.clearAll();     //前回格納したデータのクリア

                for(CustomDataStrageStructure::List<Actuator *>::Iterator itr = actuators_with_task_.begin();
                    itr != actuators_with_task_.end(); itr++)
                {
                    return_data.append((*itr)->executeTask()) ;
                    //Serial.println((*itr)->executeTask());
                }
                sender(return_data);

                time_pre = time_now;
            }
        }

        void setValue(int act_id, long value)
        {
            CustomDataStrageStructure::List<Actuator *>::Iterator itr = actuators_.begin();
            
            for(int i=0; i<act_id; i++) itr++; //イテレータをact_id回進める
            (*itr)->setValue(value);
        }

        void resetAllActuators()
        {
            for(CustomDataStrageStructure::List<Actuator *>::Iterator itr = actuators_.begin();
                itr != actuators_.end(); itr++)
            {
                (*itr)->reset(); 
            }
        }
    };
}

#endif
