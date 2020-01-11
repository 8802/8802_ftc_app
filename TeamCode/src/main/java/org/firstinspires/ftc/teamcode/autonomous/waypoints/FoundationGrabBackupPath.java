package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Config
public class FoundationGrabBackupPath implements Subroutines.ArrivalInterruptSubroutine {
    public static double BACKUP_MS = 600;
    public static double WAIT_MS = 300;

    ElapsedTime startTime;
    boolean loweredLatches;

    public FoundationGrabBackupPath() {
        startTime = null;
        loweredLatches = false;
    }

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        if (startTime == null) {
            startTime = new ElapsedTime();
            robot.setPowers(new MecanumPowers(-0.3, -0.3, -0.3, -0.3));
        } else if (!loweredLatches && startTime.milliseconds() > BACKUP_MS) {
            // Set foundation latches down
            robot.setPowers(MecanumUtil.STOP);
            robot.leftFoundationLatch.extend();
            robot.rightFoundationLatch.extend();

            // Block is already grabbed, so we can flip out
            robot.blockFlipper.normExtend();
            robot.actionCache.add(new DelayedSubroutine(500, Subroutines.OPEN_CLAW, "SKYSTONE1DEPOSITNOCHECK"));
            robot.actionCache.add(new DelayedSubroutine(750, Subroutines.LIFT_A_LITTLE, "SKYSTONE1DEPOSIT"));
            robot.actionCache.add(new DelayedSubroutine(1250, Subroutines.SET_FLIPPER_INTAKING, "SKYSTONE1DEPOSIT"));
            robot.actionCache.add(new DelayedSubroutine(1250, Subroutines.LOWER_LIFT_TO_GRABBING, "SKYSTONE1DEPOSITEND"));
            loweredLatches = true;

        } else if (startTime.milliseconds() > BACKUP_MS + WAIT_MS) {
            return true;
        }
        return false;
    }
}
