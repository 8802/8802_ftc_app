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
    Alliance alliance;

    public DepositUntilSuccessful(Alliance alliance) {
        this.alliance = alliance;
        this.timer = null;
    }

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        if (timer == null) {
            robot.setPowers(MecanumUtil.STOP);
            timer = new ElapsedTime();
            Subroutines.SMART_DROP_BLOCK.runOnce(robot);
        }

        if (timer.milliseconds() > 2500 && !robot.hasBlockInTray()) {
            return true;
        }

        if (timer.milliseconds() > 3000 && robot.hasBlockInTray()) {
            timer.reset();
            replaceBlock(robot);
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
        robot.actionCache.add(new DelayedSubroutine(2150, Subroutines.LIFT_TO_ZERO));
    }
}
