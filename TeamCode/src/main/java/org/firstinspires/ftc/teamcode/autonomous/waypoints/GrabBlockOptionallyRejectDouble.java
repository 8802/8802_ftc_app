package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

import java.util.concurrent.Delayed;

@Config
public class GrabBlockOptionallyRejectDouble implements Subroutines.ArrivalInterruptSubroutine {
    public static int ITERATIONS_BEFORE_CONTINUE = 5;

    int noBlockIterations;
    Subroutines.OnceOffSubroutine grabBlock;

    public GrabBlockOptionallyRejectDouble() {
        this(null);
    }

    public GrabBlockOptionallyRejectDouble(Subroutines.OnceOffSubroutine grabBlock) {
        this.grabBlock = grabBlock;
        noBlockIterations = -1;
    }

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        if (noBlockIterations == -1) {
            noBlockIterations = 0;
            Subroutines.GRAB_INTAKED_BLOCK.runOnce(robot);
            if (grabBlock != null) {
                // TODO ensure this doesn't lift before we cross the bridge
                grabBlock.runOnce(robot);
            }
        }
        if (robot.hasBlockInClaws()) {
            robot.setIntakePower(-1);
            noBlockIterations = 0;
        } else {
            noBlockIterations += 1;
        }

        if (noBlockIterations > ITERATIONS_BEFORE_CONTINUE) {
            robot.setIntakePower(1);
            robot.actionCache.add(new DelayedSubroutine(500, Subroutines.STOP_INTAKE));
            return true;
        } else {
            return false;
        }
    }
}
