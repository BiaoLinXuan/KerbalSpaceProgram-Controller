package KberalController;

import java.io.IOException;

import KberalController.Algorithms.PIDController.PIDController_Delta;
import KberalController.Vessel_control.Altitude_Holder;
import krpc.client.Connection;
import krpc.client.RPCException;
import krpc.client.Stream;
import krpc.client.StreamException;
import krpc.client.services.SpaceCenter;

public class Test {

    public static void main(String[] args) throws IOException, RPCException, InterruptedException, StreamException {

        Altitude_Holder altitude_holder;

        //创建服务器连接
        try (Connection connection = Connection.newInstance()) {

            SpaceCenter spaceCenter = SpaceCenter.newInstance(connection);//太空中心

            SpaceCenter.Vessel vessel = spaceCenter.getActiveVessel();

            altitude_holder = new Altitude_Holder(connection, vessel, 200);
            altitude_holder.engage();
            altitude_holder.join();


        }
    }
}
