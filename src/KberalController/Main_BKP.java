package KberalController;

import KberalController.Algorithms.PIDController.PIDController_Position;
import krpc.client.Connection;
import krpc.client.RPCException;
import krpc.client.StreamException;
import krpc.client.services.SpaceCenter;
import krpc.client.services.SpaceCenter.Vessel;

import java.io.IOException;

public class Main_BKP {
    public static void main(String[] args) throws IOException, RPCException, StreamException, InterruptedException {

        //PID控制器
        PIDController_Position pid_speed = new PIDController_Position(1, 0.01, 0.1);
        PIDController_Position pid_altitude = new PIDController_Position(0.1, 0.0001, 1);

        //创建服务器连接
        try (Connection connection = Connection.newInstance()) {

            SpaceCenter spaceCenter = SpaceCenter.newInstance(connection);                  //太空中心

            Vessel vessel = spaceCenter.getActiveVessel();                                  //当前的飞船

            SpaceCenter.ReferenceFrame refframe = vessel.getOrbit().getBody().getReferenceFrame();      //参考坐标系

            SpaceCenter.Control control = vessel.getControl();                              //飞船控制对象
            control.setSAS(true);                                                           //使能SAS

            SpaceCenter.AutoPilot autoPilot = vessel.getAutoPilot();                        //飞船自动驾驶对象
            autoPilot.targetPitchAndHeading(90, 90);                           //设置目标角度
            autoPilot.engage();                                                             //使能自动驾驶

            Thread.sleep(500);

            SpaceCenter.Flight flight = vessel.flight(vessel.getOrbit().getBody().getReferenceFrame());

            while (true) {
                double altitude = flight.getMeanAltitude();                    //高度
                double speed = -flight.getVelocity().getValue2();             //速度

                double altitude_out = pid_altitude.calc_pid(200, altitude);             //高度环输出
                double speed_out = pid_speed.calc_pid(altitude_out, speed) ;           //速度环输出

                if (speed_out > 1) speed_out = 1;                                           //限制油门
                if (speed_out < 0) speed_out = 0;

                control.setThrottle((float) speed_out);                                     //设置油门

                System.out.printf("altitude %.2f  speed %.2f  altitudePID %.2f  speedPID %.2f", altitude, speed, altitude_out, speed_out);
                System.out.println();
                Thread.sleep(10);
            }
        }
    }
}