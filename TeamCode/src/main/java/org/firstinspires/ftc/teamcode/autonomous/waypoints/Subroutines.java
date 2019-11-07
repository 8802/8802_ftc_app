package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.elements.SkystoneState;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

import java.util.concurrent.Delayed;
import java.util.concurrent.TimeUnit;

@Config
public class Subroutines {
    public static int LIFT_RAISE_AMOUNT = 800;

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

    public static final OnceOffSubroutine SET_FOUNDATION_LATCHES_DOWN = (robot) -> {
        robot.leftFoundationLatch.extend();
        robot.rightFoundationLatch.extend();
    };

    public static final OnceOffSubroutine SET_FOUNDATION_LATCHES_UP = (robot) -> {
        robot.leftFoundationLatch.retract();
        robot.rightFoundationLatch.retract();
    };

    public static final OnceOffSubroutine SET_FOUNDATION_LATCHES_OUT = (robot) -> {
        robot.leftFoundationLatch.servo.setPosition(SkystoneHardware.FOUNDATION_LATCH_OUT);
        robot.rightFoundationLatch.servo.setPosition(SkystoneHardware.FOUNDATION_LATCH_OUT);
    };

    public static final OnceOffSubroutine LIFT_A_LITTLE = (robot) -> {
        robot.pidLift.changePosition(LIFT_RAISE_AMOUNT);
    };

    public static final OnceOffSubroutine LOWER_A_LITTLE = (robot) -> {
        robot.pidLift.changePosition(-LIFT_RAISE_AMOUNT);
    };

    public static final OnceOffSubroutine LIFT_TO_ZERO = (robot) -> {
        robot.pidLift.goToMin();
    };

    public static final OnceOffSubroutine LOWER_FLIPPER_LOW = (robot) -> {
        robot.blockFlipper.setPosition(0.2, 0.18);
    };

    public static final RepeatedSubroutine CHECK_BLOCK_GRAB = (robot) -> robot.intakeCurrentQueue.hasBlock();

    /* When this is called, we assume the flipper is in intaking position and the block grabber is
    open. TODO verify that this is the case.
     */

    public static final OnceOffSubroutine GRAB_INTAKED_BLOCK = (robot) -> {
        robot.blockFlipper.readyBlockGrab();
        robot.blockGrabber.extend(); // Grab the block
        robot.actionCache.add(new DelayedSubroutine(600, Subroutines.SET_FLIPPER_DRIVING));
    };

    public static final OnceOffSubroutine SMART_DROP_BLOCK = (robot) -> {
        robot.blockFlipper.normExtend();
        robot.blockGrabber.retract();
        robot.actionCache.add(new DelayedSubroutine(250, Subroutines.LIFT_A_LITTLE));
        robot.actionCache.add(new DelayedSubroutine(750, Subroutines.SET_FLIPPER_INTAKING));
        robot.actionCache.add(new DelayedSubroutine(750, Subroutines.LIFT_TO_ZERO));
    };

    public static final OnceOffSubroutine SMART_DROP_BLOCK_WITH_LATCHES = (robot) -> {
        SET_FOUNDATION_LATCHES_DOWN.runOnce(robot);
        SMART_DROP_BLOCK.runOnce(robot);
    };

    public static final OnceOffSubroutine DROP_BLOCK_AND_RETRACT = (robot) -> {
        robot.blockGrabber.retract();
        robot.leftFoundationLatch.retract();
        robot.rightFoundationLatch.retract();
        robot.actionCache.add(new DelayedSubroutine(500, SET_FLIPPER_INTAKING));
        robot.actionCache.add(new DelayedSubroutine(1000, ENABLE_INTAKE));
    };
}