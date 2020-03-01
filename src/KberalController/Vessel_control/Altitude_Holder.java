package KberalController.Vessel_control;

/*
    高度保持器
    使用串级PID控制器保持航天器高度
 */

import KberalController.Algorithms.PIDController.PIDController_Delta;
import krpc.client.Connection;
import krpc.client.RPCException;
import krpc.client.Stream;
import krpc.client.StreamException;
import krpc.client.services.SpaceCenter;

public class Altitude_Holder extends Thread {

    private final double default_speed_p = 1;   //速度环P
    private final double default_speed_i = 1;   //速度环I
    private final double default_speed_d = 1;   //速度环D

    private final double default_altitude_p = 0.4;    //高度环P
    private final double default_altitude_i = 0.005;    //高度环I
    private final double default_altitude_d = 0.1;    //高度环D

    private Connection connection;          //与服务器的连接实例
    private SpaceCenter.Vessel vessel;      //要控制的飞船
    private double set_altitude;            //设置保持高度

    private boolean engage = false;         //启动标志
    private PIDController_Delta speed_pid;    //速度环PID
    private PIDController_Delta altitude_pid;    //高度环PID


    /*
        构造函数
        输入：与服务器连接实例，要控制的飞船,保持的高度
     */
    public Altitude_Holder(Connection connection, SpaceCenter.Vessel vessel, double set_altitude) {
        this.connection = connection;
        this.vessel = vessel;
        this.set_altitude = set_altitude;

        speed_pid = new PIDController_Delta(default_speed_p, default_speed_i, default_speed_d);//速度环PID，输入速度，输出加速度
        speed_pid.set_outputRange(-50, 50);         //输出加速度范围：-50到50

        altitude_pid = new PIDController_Delta(default_altitude_p, default_altitude_i, default_altitude_d);//高度PID，输入高度，输出速度
        altitude_pid.set_outputRange(-30, 30);      //输出速度范围：-30到30
    }


    /*
        构造函数
        输入：与服务器连接实例，要控制的飞船，保持的高度，最大速度，最大加速度
     */
    public Altitude_Holder(Connection connection, SpaceCenter.Vessel vessel, double set_altitude, double max_speed, double max_acceleration) {
        this.connection = connection;
        this.vessel = vessel;
        this.set_altitude = set_altitude;

        speed_pid = new PIDController_Delta(default_speed_p, default_speed_i, default_speed_d);         //速度环PID，输入速度，输出加速度
        speed_pid.set_outputRange(-max_acceleration, max_acceleration); //输出加速度范围：-max_acceleration 到 max_acceleration

        altitude_pid = new PIDController_Delta(default_altitude_p, default_altitude_i, default_altitude_d);//高度PID，输入高度，输出速度
        altitude_pid.set_outputRange(-max_speed, max_speed);  //输出速度范围：-max_speed 到 max_speed
    }


    /*
        更改设置高度
        参数：设置的高度
     */
    public void set_hold_altitude(double set_altitude) {
        //更改设置高度
        this.set_altitude = set_altitude;
    }


    /*
        更改PID参数
        参数：速度环p系数，速度环i系数，速度环d系数，高度环p系数，高度环i系数，高度环d系数
     */
    public void update_parameter(double speed_kp, double speed_ki, double speed_kd, double altitude_kp, double altitude_ki, double altitude_kd) {
        //更改PID参数
        speed_pid.update_parameter(speed_kp, speed_ki, speed_kd);
        altitude_pid.update_parameter(altitude_kp, altitude_ki, altitude_kd);
    }


    /*
        更改PID输出范围
        参数：最小速度，最大速度，最小加速度，最大加速度
     */
    public void update_speedAndAccelRange(double speed_min, double speed_max, double acceleration_min, double acceleration_max) {
        //更改PID输出范围
        altitude_pid.set_outputRange(speed_min, speed_max);                 //距离环输出是速度
        speed_pid.set_outputRange(acceleration_min, acceleration_max);      //速度环输出是加速度
    }


    /*
        使能高度保持
     */
    public void engage() {
        this.engage = true;
        this.start();
    }


    /*
        断开高度保持
     */
    public void disengage() {
        this.engage = false;
    }


    /*
        覆写的线程运行函数
     */
    @Override
    public void run() {
        //通信流
        Stream<Float> mass_stream = null, availableThrust_stream = null, surfaceGravity_stream = null;     //质量，最大推力，重力加速度
        Stream<SpaceCenter.Flight> flight_stream = null;    //飞行对象，内含速度和高度

        try {
            //创建流
            mass_stream = connection.addStream(vessel, "getMass");
            availableThrust_stream = connection.addStream(vessel, "getAvailableThrust");
            surfaceGravity_stream = connection.addStream(vessel.getOrbit().getBody(), "getSurfaceGravity");
            flight_stream = connection.addStream(vessel, "flight", vessel.getOrbit().getBody().getReferenceFrame());

        } catch (StreamException | RPCException e) {
            //异常处理
            System.out.printf("惊了，线程 %s 竟然创建不了流\r\n", this.getName());
            e.printStackTrace();
            System.exit(-1);
        }


        //反复执行PID运算，直到断开高度保持
        try {
            while (engage) {
                //读取流
                SpaceCenter.Flight flight = flight_stream.get();
                double mass = mass_stream.get();                                    //当前质量
                double max_thrust = availableThrust_stream.get();                   //当前最大推力
                double gravity = surfaceGravity_stream.get();                       //当前重力加速度
                double altitude = flight.getMeanAltitude();                         //当前高度

                //速度矢量
                double velocity_x = flight.getVelocity().getValue0();
                double velocity_y = flight.getVelocity().getValue1();
                double velocity_z = flight.getVelocity().getValue2();
                //速度的模
                double velocity_mod = Math.sqrt(Math.pow(velocity_x, 2) + Math.pow(velocity_y, 2) + Math.pow(velocity_z, 2));

                //通过垂直速度判断速度的方向
                if (flight.getVelocity().getValue0() < 0) velocity_mod = -velocity_mod;

                //速度环增加偏置，抵消重力影响
                speed_pid.set_outputBias(gravity);

                //PID计算
                double altitude_out = altitude_pid.calc_pid(set_altitude, altitude);
                double speed_out = speed_pid.calc_pid(altitude_out, velocity_mod);

                double throttle = 0;
                //防止由于最大推力是0导致的异常
                if (max_thrust != 0) throttle = (speed_out * mass)/max_thrust;

                vessel.getControl().setThrottle((float) throttle);

                Thread.sleep(20);
            }

        } catch (RPCException | StreamException | InterruptedException e) {
            //异常处理
            System.out.printf("淦，线程 %s 读取流的时候崩了\r\n", this.getName());
            e.printStackTrace();
            System.exit(-1);
        }


        //断开了高度保持，关闭所有流
        try {
            //关闭流
            mass_stream.remove();
            availableThrust_stream.remove();
            surfaceGravity_stream.remove();
            flight_stream.remove();

        } catch (RPCException e) {
            //异常处理
            System.out.printf("MDZZ,线程 %s 的流又关不掉了\n", this.getName());
            e.printStackTrace();
            System.exit(-1);
        }

    }

}
