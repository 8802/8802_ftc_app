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

    public DepositUntilSuccessful() {
        this.timer = null;
        this.attempt = 0;
    }

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        if (timer == null) {
            robot.setPowers(new MecanumPowers(-0.3, 0, 0));
            timer = new ElapsedTime();
            Subroutines.SMART_DROP_BLOCK.runOnce(robot);
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
        robot.actionCache.add(new DelayedSubroutine(2150, Subroutines.LIFT_TO_LAYER_ZERO));
    }
}
