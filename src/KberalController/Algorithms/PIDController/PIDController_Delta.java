package KberalController.Algorithms.PIDController;

/*
    增量式PID控制器
 */

public class PIDController_Delta {

    public double kp, ki, kd;   //PID系数
    public double[] err_storage = new double[5];    //多次的误差

    public double p_max = Double.MAX_VALUE;//比例上限
    public double i_max = Double.MAX_VALUE;//积分上限
    public double d_max = Double.MAX_VALUE;//微分上限

    public double out_max = Double.MAX_VALUE;//输出上限
    public double out_min = Double.MIN_VALUE;//输出下限

    public double bias = 0;     //输出偏置


    /*
        构造函数
     */
    public PIDController_Delta() {
        //清空积分
        reset_integral();
    }


    /*
        构造函数
        输入：p系数，i系数，d系数
     */
    public PIDController_Delta(double kp, double ki, double kd) {
        //设定PID系数
        update_parameter(kp, ki, kd);
        //清空积分
        reset_integral();
    }


    /*
        构造函数
        输入：p系数，i系数，d系数,p计算最大值，i计算最大值，d计算最大值
     */
    public PIDController_Delta(double kp, double ki, double kd, double p_max, double i_max, double d_max) {
        //设定PID系数
        update_parameter(kp, ki, kd);
        //设定计算最大值
        set_pidMaxValue(p_max, i_max, d_max);
        //清空积分
        reset_integral();
    }


    /*
        构造函数
        输入：p系数，i系数，d系数,p计算最大值，i计算最大值，d计算最大值，输出结果最大值，输出结果最小值，输出结果最小值偏置
     */
    public PIDController_Delta(double kp, double ki, double kd, double p_max, double i_max, double d_max, double out_min, double out_max, double bias) {
        //设定PID系数
        update_parameter(kp, ki, kd);
        //设定计算最大值
        set_pidMaxValue(p_max, i_max, d_max);
        //设定输出结果范围
        set_outputRange(out_min, out_max);
        //设定输出结果偏置
        set_outputBias(bias);
        //清空积分
        reset_integral();
    }


    /*
        更新PID参数
        输入：p系数，i系数，d系数
     */
    public void update_parameter(double kp, double ki, double kd) {
        //更新PID系数
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }


    /*
        分别设置PID输出的最大值
        输入：p计算最大值，i计算最大值，d计算最大值
     */
    public void set_pidMaxValue(double p_max, double i_max, double d_max) {
        //分别设置PID输出的最大值
        this.p_max = p_max;
        this.i_max = i_max;
        this.d_max = d_max;
    }


    /*
        设置PID计算后输出结果的偏置
        输入：输出结果的偏置
     */
    public void set_outputBias(double bias) {
        //设置PID计算后输出结果的偏置
        this.bias = bias;
    }


    /*
        设置PID计算后输出结果的范围
        输入：输出结果最大值，输出结果最小值
     */
    public void set_outputRange(double out_min, double out_max) {
        //设置PID计算后输出结果的范围
        this.out_min = out_min;
        this.out_max = out_max;
    }


    /*
        清空积分值
     */
    public void reset_integral() {
        for (double err_each : this.err_storage) {
            err_each = 0;
        }
    }


    /*
        进行PID计算
        输入：设定值，实际值
        输出：计算结果
     */
    public double calc_pid(double set, double actual) {

        //移位误差
        err_storage[4] = err_storage[3];
        err_storage[3] = err_storage[2];
        err_storage[2] = err_storage[1];
        err_storage[1] = err_storage[0];
        err_storage[0] = set - actual;;

        //P控制
        double p = err_storage[0] * kp;     //比例控制
        if (p > p_max) p = p_max;           //比例控制限制

        //I控制
        double err_sum = 0;                 //误差积分
        for (double err_each : err_storage) {
            err_sum += err_each;
        }
        double i = err_sum * ki;            //积分控制
        if (i > i_max) i = i_max;           //积分控制限制

        //D控制
        double d = (err_storage[0] - err_storage[1]) * kd;//微分控制
        if (d > d_max) d = d_max;           //微分控制限制

        //输出限制
        double out = p + i - d + bias;
        if (out > out_max) out = out_max;
        if (out < out_min) out = out_min;

        return out;
    }
}
