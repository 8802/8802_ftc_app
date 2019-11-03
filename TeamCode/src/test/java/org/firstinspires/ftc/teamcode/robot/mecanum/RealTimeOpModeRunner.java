package org.firstinspires.ftc.teamcode.robot.mecanum;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.simulator.RXThread;
import org.firstinspires.ftc.simulator.SimulatedOpModeFactory;
import org.firstinspires.ftc.simulator.TXHandler;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.auto.*;
import org.firstinspires.ftc.teamcode.robot.mecanum.teleop.SkystoneRobotCentric;
import org.junit.jupiter.api.Test;

import java.io.IOException;

class RealTimeOpModeRunner {
    final static double FRAMERATE = 60;

    @Test
    void testOpMode() throws IOException {
        SimulatedOpModeFactory simOpMode = new SimulatedOpModeFactory(BlindFourBlockBlue.class);
        simOpMode.opMode.start();
        TXHandler udpServer = new TXHandler(-1);
        RXThread udpClient = new RXThread(simOpMode.opMode.gamepad1, simOpMode.opMode.gamepad2);
        udpClient.start();
        ElapsedTime time = new ElapsedTime();

        while(time.seconds() < 120 && !simOpMode.stopRequested) {
            simOpMode.opMode.loop();
            simOpMode.robot.elapse(1 / FRAMERATE);
            udpServer.sendMessage(simOpMode.robot.pose());

            try {
                Thread.sleep((long) (1000 / FRAMERATE));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        simOpMode.robot.velocity = new Pose(0,0, 0);
        simOpMode.robot.setPowers(MecanumUtil.STOP);
        udpServer.sendMessage(simOpMode.robot.pose());
        udpClient.kill();
    }
}