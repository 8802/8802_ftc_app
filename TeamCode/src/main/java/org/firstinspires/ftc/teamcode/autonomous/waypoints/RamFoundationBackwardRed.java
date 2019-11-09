package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.elements.Alliance;
import org.firstinspires.ftc.teamcode.common.elements.SkystoneState;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Config
public class RamFoundationBackwardRed implements Subroutines.ArrivalInterruptSubroutine {

    ElapsedTime timer;
    Alliance alliance;
    State state;

    enum State {
        NOT_RUNNING, SHOVING, REDOING_PLACEMENT
    }

    public RamFoundationBackwardRed(Alliance alliance) {
        this.alliance = alliance;
        this.timer = null;
        this.state = State.NOT_RUNNING;
    }

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        if (state == State.NOT_RUNNING) {
            robot.setPowers(new MecanumPowers(-1, -0.6, -1, -0.6));
            timer = new ElapsedTime();
            state = State.SHOVING;
        }

        if (state == State.SHOVING) {
            if (robot.localizer.velocity().radius() < 1 ||
                    timer.milliseconds() > 2000) {
                robot.leftFoundationLatch.retract();
                robot.rightFoundationLatch.retract();
                /* Verify placement */
                if (robot.hasBlockInTray()) {
                    state = State.REDOING_PLACEMENT;
                    robot.setPowers(MecanumUtil.STOP);
                    robot.opModeState = SkystoneHardware.OpModeState.ERRORS;
                    timer.reset();
                    replaceBlock(robot);
                } else {
                    return true;
                }
            }
        }

        if (state == State.REDOING_PLACEMENT) {
            if (timer.milliseconds() > 2500) {
                if (robot.hasBlockInTray()) {
                    timer.reset();
                    replaceBlock(robot);
                } else {
                    return true;
                }
            }
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
