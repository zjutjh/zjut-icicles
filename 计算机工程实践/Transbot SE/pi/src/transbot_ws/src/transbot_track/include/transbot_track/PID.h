
class PID {
public:
    float kp;
    float ki;
    float kd;
    float targetpoint;
    // 上一次的误差
    //Last time error
    float prevError;
    // 积分
    //integral
    float intergral;
    // 微分
    //differential
    float derivative;

    PID(float kp, float ki, float kd);

    void Set_PID(float kp, float ki, float kd);

    /**
     * pid calculation function pid的计算函数
     * @param target  目标值
     * @param current 当前值
     * @return  pwm
     */
    float compute(float target, float current);

    /**
     *  重置所有的误差: 当设置的速度 和 上一次不一样
     *  Reset all errors: When the set speed is different from the last time
     */
    void reset();
};
