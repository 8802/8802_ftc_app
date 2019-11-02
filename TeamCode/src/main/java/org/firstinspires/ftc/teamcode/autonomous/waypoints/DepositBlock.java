package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Config
public class DepositBlock implements Subroutines.ArrivalInterruptSubroutine {
    private enum DEPOSIT_STATE {
        PLACING, RELEASING, RETRACTING
    }
    public static double AUTO_DEPOSIT_FLIPPER_POSITION = 0.7;
    public static double PAUSE_MS = 300;
    public static double RELEASE_MS = 200;
    public static double LIFT_MS = 500;

    private ElapsedTime timeSinceStart;

    public DepositBlock(MecanumPowers direction) {
    }

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        if (timeSinceStart == null) {
            timeSinceStart = new ElapsedTime(); // Start the timer
            robot.blockFlipper.setPosition(AUTO_DEPOSIT_FLIPPER_POSITION);
        }
        if (timeSinceStart.milliseconds() > PAUSE_MS) {
            //robot.blockGrabber
        };
        return false;
    }
}
