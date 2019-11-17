package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Config
public class OptionallyRejectBlock implements Subroutines.ArrivalInterruptSubroutine {
    public static double JOLT_MS = 500;
    public static double PAUSE_MS = 300;
    public static double MAX_JOLTS = 1;

    ElapsedTime elapsedTime;
    ElapsedTime timeSinceDoubleBlock;

    public OptionallyRejectBlock() {
        elapsedTime = null;
    }

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        if (elapsedTime == null) {
            robot.setPowers(MecanumUtil.STOP);
            elapsedTime = new ElapsedTime();
            timeSinceDoubleBlock = new ElapsedTime();
        } else if (elapsedTime.milliseconds() > 500) {
            if (!robot.hasBlockInClaws()) {
                if (timeSinceDoubleBlock.milliseconds() > 500) {
                    robot.setIntakePower(0);
                    return true;
                }
            } else {
                timeSinceDoubleBlock.reset();
                robot.setIntakePower(-1);
            }
        }
        return false;
    }
}
