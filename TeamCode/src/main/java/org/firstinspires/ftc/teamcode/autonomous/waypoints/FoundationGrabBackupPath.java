package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.SimpleLift;

@Config
public class FoundationGrabBackupPath implements Subroutines.ArrivalInterruptSubroutine {
    public static double BACKUP_MS = 450;
    public static double WAIT_MS = 600;

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
            robot.actionCache.add(new DelayedSubroutine(750, (r) -> {
                r.pidLift.left.setTargetPosition(Subroutines.LIFT_RAISE_AMOUNT);
                r.pidLift.right.setTargetPosition(Subroutines.LIFT_RAISE_AMOUNT);
                r.pidLift.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                r.pidLift.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                r.pidLift.left.setPower(1);
                r.pidLift.right.setPower(1);
            }, "SKYSTONE1DEPOSIT"));
            robot.actionCache.add(new DelayedSubroutine(1250, Subroutines.SET_FLIPPER_INTAKING, "SKYSTONE1DEPOSIT"));
            robot.actionCache.add(new DelayedSubroutine(1250, (r) -> {
                r.pidLift.left.setTargetPosition(SimpleLift.GRABBING);
                r.pidLift.right.setTargetPosition(SimpleLift.GRABBING);
                r.pidLift.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                r.pidLift.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                r.pidLift.left.setPower(1);
                r.pidLift.right.setPower(1);
            }, "SKYSTONE1DEPOSITEND"));
            loweredLatches = true;
            startTime.reset();

        } else if (loweredLatches && startTime.milliseconds() > WAIT_MS) {
            return true;
        }
        return false;
    }
}
