package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.elements.Alliance;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.SimpleLift;

public class CloseDepositUntilSuccessful implements Subroutines.RepeatedSubroutine {

    ElapsedTime timer;
    int attempt;

    public CloseDepositUntilSuccessful() {
        this.timer = null;
        this.attempt = 0;
    }

    @Override
    public boolean runLoop(SkystoneHardware robot, PurePursuitPath path) {
        if (timer == null) {
            timer = new ElapsedTime();
            robot.actionCache.add(new DelayedSubroutine(150 + 100, Subroutines.SET_FLIPPER_MAX_EXTEND));
            robot.actionCache.add(new DelayedSubroutine(425 + 150, (r) -> {
                r.pidLift.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                r.pidLift.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                r.pidLift.left.setPower(0.8);
                r.pidLift.right.setPower(0.8);
            }));
            robot.actionCache.add(new DelayedSubroutine(600 + 150, Subroutines.OPEN_CLAW));
            robot.actionCache.add(new DelayedSubroutine(950 + 150, Subroutines.SET_FLIPPER_INTAKING));
            robot.actionCache.add(new DelayedSubroutine(950 + 150, (r) -> {
                r.pidLift.left.setTargetPosition(SimpleLift.GRABBING);
                r.pidLift.right.setTargetPosition(SimpleLift.GRABBING);
                r.pidLift.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                r.pidLift.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                r.pidLift.left.setPower(1);
                r.pidLift.right.setPower(1);
            }));
            attempt = 1;
        }

        if (attempt == 1) {
            if (timer.milliseconds() > 1200 && !robot.hasBlockInTray()) {
                return true;
            }
        } else {
            if (timer.milliseconds() > 4000
                    && !robot.hasBlockInTray()) {
                return true;
            }
        }

        if (timer.milliseconds() > 3000 && robot.hasBlockInTray()) {
            timer.reset();
            replaceBlock(robot);
            attempt += 1;
        }

        return false;
    }

    private void replaceBlock(SkystoneHardware robot) {
        robot.blockFlipper.readyBlockGrab();
        robot.blockGrabber.extend(); // Grab the block
        robot.pidLift.left.setTargetPosition(SimpleLift.GRABBING);
        robot.pidLift.right.setTargetPosition(SimpleLift.GRABBING);
        robot.pidLift.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.pidLift.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.pidLift.left.setPower(1);
        robot.pidLift.right.setPower(1);
        robot.actionCache.add(new DelayedSubroutine(600, Subroutines.SET_FLIPPER_NORM_EXTEND));
        robot.actionCache.add(new DelayedSubroutine(1400, Subroutines.OPEN_CLAW));
        robot.actionCache.add(new DelayedSubroutine(1650, (r) -> {
            r.pidLift.left.setTargetPosition(Subroutines.LIFT_RAISE_AMOUNT);
            r.pidLift.right.setTargetPosition(Subroutines.LIFT_RAISE_AMOUNT);
        }));
        robot.actionCache.add(new DelayedSubroutine(2150, Subroutines.SET_FLIPPER_INTAKING));
        robot.actionCache.add(new DelayedSubroutine(2150, (r) -> {
            r.pidLift.left.setTargetPosition(SimpleLift.GRABBING);
            r.pidLift.right.setTargetPosition(SimpleLift.GRABBING);
        }));
    }
}
