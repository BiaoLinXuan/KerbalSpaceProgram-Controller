package KberalController;

import KberalController.Algorithms.PIDController.PIDController_Delta;
import KberalController.Vessel_control.Altitude_Holder;
import krpc.client.Connection;
import krpc.client.RPCException;
import krpc.client.Stream;
import krpc.client.StreamException;
import krpc.client.services.SpaceCenter;
import krpc.client.services.SpaceCenter.Vessel;

import java.io.IOException;

public class Main {
    public static void main(String[] args) throws IOException, RPCException, InterruptedException, StreamException {

        PIDController_Delta speed_pid = new PIDController_Delta(0.3, 0, 0);
        //创建服务器连接
        try (Connection connection = Connection.newInstance()) {

            SpaceCenter spaceCenter = SpaceCenter.newInstance(connection);//太空中心

            Vessel vessel = spaceCenter.getActiveVessel();
            Stream<SpaceCenter.Flight> a = connection.addStream(spaceCenter.getActiveVessel(),"getMass");
            a.setRate(50);
            while (true){
                synchronized (a.getCondition()){
                    System.out.println(a.get());
                    a.waitForUpdate();
                }
            }

//            vessel.getControl().setRCS(true);
//            vessel.getControl().setSAS(true);
//
//            while (true) {
//                double distance = get_rev_distance(vessel);
//                double altitude = vessel.flight(vessel.getOrbit().getBody().getReferenceFrame()).getMeanAltitude();
//                System.out.printf("distance %f   altitude %f\r\n", distance, altitude);
//                if (distance > altitude && altitude<10000){
//                    System.out.printf("full power\r\n");
//                    vessel.getControl().setThrottle(1);
//                    break;
//                }
//                Thread.sleep(10);
//            }
//            //等待落地
//            while (vessel.getSituation() != SpaceCenter.VesselSituation.LANDED) Thread.sleep(10);
//            vessel.getControl().setThrottle(0);
        }
    }

    public static double get_rev_distance(Vessel vessel) throws RPCException {
        double mass = vessel.getMass();                                     //当前质量
        double thrust = vessel.getMaxThrust();                              //当前推力
        double gravity = vessel.getOrbit().getBody().getSurfaceGravity();   //当前重力
        double altitude = vessel.flight(vessel.getOrbit().getBody().getReferenceFrame()).getSurfaceAltitude();  //当前高度

        //速度矢量
        double velocity_x = vessel.flight(vessel.getOrbit().getBody().getReferenceFrame()).getVelocity().getValue0();
        double velocity_y = vessel.flight(vessel.getOrbit().getBody().getReferenceFrame()).getVelocity().getValue2();
        double velocity_z = vessel.flight(vessel.getOrbit().getBody().getReferenceFrame()).getVelocity().getValue2();
        //速度的模
        double velocity_mod = Math.sqrt(Math.pow(velocity_x, 2) + Math.pow(velocity_y, 2) + Math.pow(velocity_z, 2));

        double f = thrust - mass * gravity;//f = 推力 - 重力
        double a = f / mass;               //a = f/m

        double t = velocity_mod / a;       //v = at

        double distance = 0.5 * a * t * t; //x = 1/2 a t2

        return distance;
    }

}