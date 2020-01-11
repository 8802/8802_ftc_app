package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.elements.Alliance;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Config
public class DepositUntilSuccessful implements Subroutines.ArrivalInterruptSubroutine {

    ElapsedTime timer;
    int attempt;
    MecanumPowers driveDir;
    boolean overshoot;

    public DepositUntilSuccessful(MecanumPowers driveDir) {
        this(driveDir, false);
    }

    public DepositUntilSuccessful(MecanumPowers driveDir, boolean overshoot) {
        this.timer = null;
        this.attempt = 0;
        this.driveDir = driveDir;
        this.overshoot = overshoot;
    }

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        if (timer == null) {
            robot.setPowers(driveDir);
            timer = new ElapsedTime();
            robot.blockFlipper.normExtend();
            robot.actionCache.add(new DelayedSubroutine(500, Subroutines.OPEN_CLAW));
            robot.actionCache.add(new DelayedSubroutine(1250, Subroutines.SET_FLIPPER_INTAKING));
            if (overshoot) {
                robot.actionCache.add(new DelayedSubroutine(750, Subroutines.LIFT_A_FAIR_BIT));
                robot.actionCache.add(new DelayedSubroutine(1600, Subroutines.LOWER_LIFT_TO_GRABBING));
            } else {
                robot.actionCache.add(new DelayedSubroutine(750, Subroutines.LIFT_A_LITTLE));
                robot.actionCache.add(new DelayedSubroutine(1250, Subroutines.LOWER_LIFT_TO_GRABBING));
            }
            attempt = 1;
        }

        // TODO wait longer if this is our second time
        if (attempt == 1) {
            if (timer.milliseconds() > 2000 && !robot.hasBlockInTray()) {
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
