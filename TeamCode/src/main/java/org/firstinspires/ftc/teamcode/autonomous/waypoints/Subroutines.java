package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

import java.util.concurrent.Delayed;
import java.util.concurrent.TimeUnit;

@Config
public class Subroutines {
    public static int LIFT_RAISE_AMOUNT = 500;

    public interface Subroutine {}

    public interface OnceOffSubroutine extends Subroutine {
        void runOnce(SkystoneHardware robot);
    }

    public interface MetaSubroutine extends Subroutine {
        void runOnce(PurePursuitPath path);
    }

    public interface RepeatedSubroutine extends Subroutine {
        boolean runLoop(SkystoneHardware robot); // Returns whether we should advance
    }

    // Once a position is reached, if that position has an ArrivalInterruptSubroutine, we advance to
    // the next waypoint (so the waypoint with the ArrivalInterruptSubroutine is our current waypoint)
    // and we call runCycle every tick until it eventually returns true
    public interface ArrivalInterruptSubroutine extends Subroutine {
        boolean runCycle(SkystoneHardware robot); // Returns whether it's complete
    }

    public static final OnceOffSubroutine ENABLE_INTAKE = (robot) -> { robot.setIntakePower(1); };
    public static final OnceOffSubroutine STOP_INTAKE = (robot) -> { robot.setIntakePower(0); };
    public static final OnceOffSubroutine REVERSE_INTAKE = (robot) -> { robot.setIntakePower(-1); };

    public static final OnceOffSubroutine OPEN_CLAW = (robot) -> { robot.blockGrabber.retract(); };
    public static final OnceOffSubroutine CLOSE_CLAW = (robot) -> { robot.blockGrabber.extend(); };

    public static final OnceOffSubroutine SET_FLIPPER_GRABBING = (robot) -> { robot.blockFlipper.readyBlockGrab(); };
    public static final OnceOffSubroutine SET_FLIPPER_INTAKING = (robot) -> { robot.blockFlipper.readyBlockIntake(); };
    public static final OnceOffSubroutine SET_FLIPPER_DRIVING = (robot) -> { robot.blockFlipper.readyDriving(); };
    public static final OnceOffSubroutine SET_FLIPPER_MAX_OUT = (robot) -> { robot.blockFlipper.maxExtend(); };

    public static final OnceOffSubroutine LIFT_A_LITTLE = (robot) -> {
        robot.pidLift.changePosition(LIFT_RAISE_AMOUNT);
    };

    public static final OnceOffSubroutine LOWER_A_LITTLE = (robot) -> {
        robot.pidLift.changePosition(-LIFT_RAISE_AMOUNT);
    };

    public static final RepeatedSubroutine CHECK_BLOCK_GRAB = (robot) -> {
        return robot.intakeCurrentQueue.hasBlock();
    };

    /* When this is called, we assume the flipper is in intaking position and the block grabber is
    open. TODO verify that this is the case.
     */

    public static final OnceOffSubroutine AUTO_PROCESS_INTAKED_BLOCK = (robot) -> {
        CLOSE_CLAW.runOnce(robot);
        SET_FLIPPER_GRABBING.runOnce(robot);
        // Now, we'll add a delay to let this happen
        robot.actionCache.add(new DelayedSubroutine(500, SET_FLIPPER_DRIVING));
    };

    public static final OnceOffSubroutine DEPOSIT_BLOCK = (robot) -> {
        CLOSE_CLAW.runOnce(robot);
        SET_FLIPPER_GRABBING.runOnce(robot);
        // Now, we'll add a delay to let this happen
        robot.actionCache.add(new DelayedSubroutine(100, SET_FLIPPER_DRIVING));
    };
}