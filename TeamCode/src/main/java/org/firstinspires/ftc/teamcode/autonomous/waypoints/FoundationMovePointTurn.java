package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Config
public class FoundationMovePointTurn implements Subroutines.ArrivalInterruptSubroutine {
    enum Mode {
        TURNING, CHECKING, REDROPPING
    }
    public static double REDUCTION_DIST = Math.PI/3;
    public static int TIME_UNTIL_FIRST_CHECK_MS = 1000;
    public static int TIME_UNTIL_SUBSEQUENT_CHECKS_MS = 1500;

    double targetHeading;
    double allowedError;
    Mode mode;
    long checkAtTime;

    public FoundationMovePointTurn(double targetHeading, double allowedError) {
        this.targetHeading = targetHeading;
        this.allowedError = allowedError;
        this.mode = Mode.TURNING;
        this.checkAtTime = -1; // Unnecessary, but good to make this explicit
    }

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        if (mode == Mode.TURNING) {
            double currentHeading = robot.pose().heading;
            double angleToTarget = MathUtil.angleWrap(targetHeading - currentHeading);
            Pose poseTurnPower = new Pose(0, 0, angleToTarget / REDUCTION_DIST);
            robot.setPowers(new MecanumPowers(poseTurnPower));

            if (Math.abs(currentHeading - targetHeading) < allowedError) {
                mode = Mode.CHECKING;
                // We can't call this earlier or we'll slide and lose the block
                Subroutines.SMART_DROP_BLOCK.runOnce(robot);
                robot.setPowers(MecanumUtil.STOP);
                checkAtTime = ms() + TIME_UNTIL_FIRST_CHECK_MS;
            }
            return false;
        }

        if (mode == Mode.CHECKING && ms() > checkAtTime) {
            if (robot.hasBlockInTray()) {
                // We didn't grab the block successfully
                checkAtTime = ms() + TIME_UNTIL_SUBSEQUENT_CHECKS_MS;
                robot.actionCache.clear(); // Remove everything from action cache

                // Grab the block
                robot.pidLift.setLayer(0);
                robot.blockGrabber.retract(); // Open claw
                robot.blockFlipper.readyBlockGrab(); // Move flipper
                robot.actionCache.add(new DelayedSubroutine(500, Subroutines.CLOSE_CLAW)); // Close claw
                // TODO fix how we can't edit delayed subroutines within a delayed subroutine
                // which would let us just call SMART_DROP_BLOCK here
                robot.actionCache.add(new DelayedSubroutine(1000, Subroutines.SET_FLIPPER_NORM_EXTEND));
                robot.actionCache.add(new DelayedSubroutine(1000, Subroutines.OPEN_CLAW));
                robot.actionCache.add(new DelayedSubroutine(1250, Subroutines.LIFT_A_LITTLE));
                robot.actionCache.add(new DelayedSubroutine(1500, Subroutines.SET_FOUNDATION_LATCHES_UP));
                robot.actionCache.add(new DelayedSubroutine(1750, Subroutines.SET_FLIPPER_INTAKING));
                robot.actionCache.add(new DelayedSubroutine(1750, Subroutines.LIFT_TO_LAYER_ZERO));
            } else {
                // If we don't have a block in the tray, meaning we placed successfully:
                return true;
                // Don't lift the latches and don't worry about the fact we haven't finished placing
                // yet, because we're just going to back up with the foundation.
            }
        }
        return false;
    }

    private long ms() {
        return System.currentTimeMillis();
    }
}
