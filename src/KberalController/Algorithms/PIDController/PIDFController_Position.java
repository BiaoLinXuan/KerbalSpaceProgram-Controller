package KberalController.Algorithms.PIDController;

/*
    位置式带前馈的PID控制器
 */

public class PIDFController_Position extends PIDController_Position {

    public double kf;          //前馈系数
    public double kf_min = Double.MAX_VALUE;        //前馈系数下限
    public double kf_max = Double.MAX_VALUE;        //前馈系数上限


    /*
        构造函数
     */
    public PIDFController_Position() {
        super();
    }


    /*
        构造函数
        输入：p系数，i系数，d系数，f系数
     */
    public PIDFController_Position(double kp, double ki, double kd, double kf) {
        super(kp, ki, kd);                           //设定PID系数
        this.kf = kf;                                //设定F系数
    }


    /*
        构造函数
        输入：p系数，i系数，d系数，f系数，p计算最大值，i计算最大值，d计算最大值，f计算最大值
     */
    public PIDFController_Position(double kp, double ki, double kd, double kf, double p_max, double i_max, double d_max, double kf_max) {
        super(kp, ki, kd, p_max, i_max, d_max);             //设定PID系数
        this.kf = kf;                                       //设定F系数
        this.kf_max = kf_max;                               //设定F最大值
    }


    /*
        构造函数
        输入：p系数，i系数，d系数，f系数，p计算最大值，i计算最大值，d计算最大值，PID输出结果最小值，PID输出结果最大值，输出结果偏置，F输出结果最小值，F输出结果最大值
     */
    public PIDFController_Position(double kp, double ki, double kd, double kf, double p_max, double i_max, double d_max, double pid_min, double pid_max, double bias, double kf_min, double kf_max) {
        super(kp, ki, kd, p_max, i_max, d_max, pid_min, pid_max, bias);//设定PID系数
        this.kf = kf;               //设定F系数
        this.kf_min = kf_min;       //设定F最小值
        this.kf_max = kf_max;       //设定F最大值
    }


    /*
        更新PIDF参数
        输入：p系数，i系数，d系数，f系数
     */
    public void update_parameter(double kp, double ki, double kd, double kf) {
        super.update_parameter(kp, ki, kd);          //更新PID系数
        this.kf = kf;                                //更新F系数
    }


    /*
        分别设置PIDF输出的最大值
        输入：p计算最大值，i计算最大值，d计算最大值，f计算最大值
     */
    public void set_pidfMaxValue(double p_max, double i_max, double d_max, double kf_max) {
        //分别设置PID输出的最大值
        set_pidMaxValue(p_max, i_max, d_max);
        this.kf_max = kf_max;
    }


    /*
        设置PIDF计算后输出结果的范围
        输入：PID输出结果最大值，PID输出结果最小值，F输出结果最大值，F输出结果最小值
     */
    public void set_outputRange(double out_min, double out_max, double kf_min, double kf_max) {
        //设置PID计算后输出结果的范围
        set_outputRange(out_min, out_max);
        //设置F计算后输出结果的范围
        this.kf_min = kf_min;
        this.kf_max = kf_max;
    }


    /*
        进行PID计算
        输入：设定值，实际值
        输出：计算结果
     */
    public double calc_pid(double set, double actual) {
        double pid_val = super.calc_pid(set, actual);        //PID计算结果
        double f_val = set * kf;                             //前馈计算结果

        //限制f输出范围
        if (f_val > kf_max) f_val = kf_max;
        if (f_val < kf_min) f_val = kf_min;

        return pid_val + f_val;
    }
}
