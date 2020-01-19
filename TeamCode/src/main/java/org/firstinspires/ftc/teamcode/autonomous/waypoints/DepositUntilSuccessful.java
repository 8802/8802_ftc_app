package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.elements.Alliance;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Config
public class DepositUntilSuccessful implements Subroutines.RepeatedSubroutine {

    public enum DepositHeight {
        LOW, HIGH
    }

    ElapsedTime timer;
    int attempt;
    DepositHeight height;

    public DepositUntilSuccessful() {
        this(DepositHeight.LOW);
    }

    public DepositUntilSuccessful(DepositHeight height) {
        this.timer = null;
        this.attempt = 0;
        this.height = height;
    }

    @Override
    public boolean runLoop(SkystoneHardware robot) {
        if (timer == null) {
            timer = new ElapsedTime();
            robot.actionCache.add(new DelayedSubroutine(150, Subroutines.SET_FLIPPER_MAX_EXTEND));
            if (height == DepositHeight.HIGH) {
                robot.actionCache.add(new DelayedSubroutine(325, (r) -> {r.pidLift.lift.setPower(0.8);}));
            } else {
                robot.actionCache.add(new DelayedSubroutine(450, (r) -> {r.pidLift.lift.setPower(0.8);}));
            }
            robot.actionCache.add(new DelayedSubroutine(600, Subroutines.OPEN_CLAW));
            robot.actionCache.add(new DelayedSubroutine(950, Subroutines.SET_FLIPPER_INTAKING));
            robot.actionCache.add(new DelayedSubroutine(950, Subroutines.LOWER_LIFT_TO_GRABBING));
            attempt = 1;
        }

        if (attempt == 1) {
            if (timer.milliseconds() > 1200 && !robot.hasBlockInTray()) {
                return true;
            }
        } else {
            if (timer.milliseconds() > 3000 && !robot.hasBlockInTray()) {
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
        robot.actionCache.add(new DelayedSubroutine(600, Subroutines.SET_FLIPPER_NORM_EXTEND));
        robot.actionCache.add(new DelayedSubroutine(1400, Subroutines.OPEN_CLAW));
        robot.actionCache.add(new DelayedSubroutine(1650, Subroutines.LIFT_A_LITTLE));
        robot.actionCache.add(new DelayedSubroutine(2150, Subroutines.SET_FLIPPER_INTAKING));
        robot.actionCache.add(new DelayedSubroutine(2150, Subroutines.LOWER_LIFT_TO_GRABBING));
    }
}
