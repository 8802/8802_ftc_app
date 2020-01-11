package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Config
public class Subroutines {
    public static int LIFT_RAISE_AMOUNT = 8000;
    public static int HIGH_LIFT_RAISE_AMOUNT = 10000;
    public static double INTAKE_SPEED = 0.65;

    public interface Subroutine {}

    public interface OnceOffSubroutine extends Subroutine {
        void runOnce(SkystoneHardware robot);
    }

    public interface MetaSubroutine extends Subroutine {
        void runOnce(PurePursuitPath path, SkystoneHardware robot);
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

    public static final OnceOffSubroutine ENABLE_INTAKE = (robot) -> { robot.setIntakePower(INTAKE_SPEED); };
    public static final OnceOffSubroutine STOP_INTAKE = (robot) -> { robot.setIntakePower(0); };
    public static final OnceOffSubroutine REVERSE_INTAKE = (robot) -> { robot.setIntakePower(-1); };
    public static final OnceOffSubroutine JOLT_INTAKE = (robot) -> {
        robot.setIntakePower(1);
        robot.actionCache.add(new DelayedSubroutine(300, STOP_INTAKE));
    };


    public static final OnceOffSubroutine OPEN_CLAW = (robot) -> { robot.blockGrabber.retract(); };
    public static final OnceOffSubroutine CLOSE_CLAW = (robot) -> { robot.blockGrabber.extend(); };
    public static final OnceOffSubroutine CAPSTONE_CLAW = (robot) -> {
        robot.blockGrabber.servo.setPosition(SkystoneHardware.BLOCK_GRABBER_CAPSTONE);
    };

    public static final OnceOffSubroutine SET_FLIPPER_GRABBING = (robot) -> { robot.blockFlipper.readyBlockGrab(); };
    public static final OnceOffSubroutine SET_FLIPPER_INTAKING = (robot) -> { robot.blockFlipper.readyBlockIntake(); };
    public static final OnceOffSubroutine SET_FLIPPER_DRIVING = (robot) -> { robot.blockFlipper.readyDriving(); };
    public static final OnceOffSubroutine SET_FLIPPER_NORM_EXTEND = (robot) -> { robot.blockFlipper.normExtend(); };

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

    public static final OnceOffSubroutine SET_FANGS_DOWN = (robot) -> {
        robot.leftFang.extend();
        robot.rightFang.extend();
    };

    public static final OnceOffSubroutine SET_FANGS_UP = (robot) -> {
        robot.leftFang.retract();
        robot.rightFang.retract();
    };

    public static final OnceOffSubroutine LIFT_LEVEL_ONE = (robot) -> {
        robot.pidLift.setLayer(1);
    };

    public static final OnceOffSubroutine LIFT_LEVEL_TWO = (robot) -> {
        robot.pidLift.setLayer(2);
    };

    public static final OnceOffSubroutine LIFT_A_LITTLE = (robot) -> {
        robot.pidLift.changePosition(LIFT_RAISE_AMOUNT);
    };

    public static final OnceOffSubroutine LIFT_A_FAIR_BIT = (robot) -> {
        robot.pidLift.changePosition(HIGH_LIFT_RAISE_AMOUNT);
    };

    public static final OnceOffSubroutine LOWER_A_LITTLE = (robot) -> {
        robot.pidLift.changePosition(-LIFT_RAISE_AMOUNT);
    };

    public static final OnceOffSubroutine LOWER_A_FAIR_BIT = (robot) -> {
        robot.pidLift.changePosition(-HIGH_LIFT_RAISE_AMOUNT);
    };

    public static final OnceOffSubroutine LOWER_LIFT_TO_GRABBING = (robot) -> {
        robot.pidLift.cacheToGrabbing();
    };

    public static final OnceOffSubroutine LIFT_TO_LAYER_ZERO = (robot) -> {
        robot.pidLift.setLayer(0);
    };

    public static final OnceOffSubroutine LOWER_FLIPPER_LOW = (robot) -> {
        robot.blockFlipper.setPosition(0.2, 0.18);
    };

    public static final RepeatedSubroutine CHECK_BLOCK_GRAB = (robot) -> robot.hasBlockInClaws();

    public static final RepeatedSubroutine CHECK_BLOCK_GRAB_OR_TRAY = (robot) -> {
        boolean result = robot.hasBlockInClaws() || robot.hasBlockInTray();
        System.out.println(System.currentTimeMillis());
        if (result) {
            System.out.println("Ran CHECK_BLOCK_GRAB_WITH_TRAY and got result 'true'");
        } else {
            System.out.println("Ran CHECK_BLOCK_GRAB_WITH_TRAY and got result 'false'");
        }
        return result;
    };

    public static final OnceOffSubroutine STOP_OP_MODE_IF_DOUBLED_BLOCK = (robot) -> {
        if (robot.hasBlockInClaws()) {
            // TODO add code to stop robot
        } else {
            robot.setIntakePower(0);
        }
    };

    /* When this is called, we assume the flipper is in intaking position and the block grabber is
    open. TODO verify that this is the case.
     */

    public static final OnceOffSubroutine GRAB_INTAKED_BLOCK = (robot) -> {
        robot.blockFlipper.readyBlockGrab();
        robot.blockGrabber.extend(); // Grab the block
        robot.actionCache.add(new DelayedSubroutine(850, Subroutines.SET_FLIPPER_DRIVING));
    };

    public static final OnceOffSubroutine GRAB_INTAKED_BLOCK_AND_LIFT = (robot) -> {
        robot.blockFlipper.readyBlockGrab();
        robot.blockGrabber.extend(); // Grab the block
        robot.actionCache.add(new DelayedSubroutine(850, Subroutines.SET_FLIPPER_DRIVING));
        robot.actionCache.add(new DelayedSubroutine(850, Subroutines.LIFT_LEVEL_ONE));
    };

    public static final OnceOffSubroutine GRAB_INTAKED_BLOCK_AND_LIFT_LEVEL_2 = (robot) -> {
        robot.blockFlipper.readyBlockGrab();
        robot.blockGrabber.extend(); // Grab the block
        robot.actionCache.add(new DelayedSubroutine(850, Subroutines.SET_FLIPPER_DRIVING));
        robot.actionCache.add(new DelayedSubroutine(850, Subroutines.LIFT_LEVEL_TWO));
    };


    public static final OnceOffSubroutine GRAB_BLOCK_NO_EXTEND = (robot) -> {
        robot.blockFlipper.readyBlockGrab();
        robot.blockGrabber.extend(); // Grab the block
    };

    public static final OnceOffSubroutine SMART_DROP_BLOCK = (robot) -> {
        robot.blockFlipper.normExtend();
        robot.blockGrabber.retract();
        robot.actionCache.add(new DelayedSubroutine(250, Subroutines.LIFT_A_LITTLE));
        robot.actionCache.add(new DelayedSubroutine(750, Subroutines.SET_FLIPPER_INTAKING));
        robot.actionCache.add(new DelayedSubroutine(750, Subroutines.LOWER_LIFT_TO_GRABBING));
    };

    public static final OnceOffSubroutine GRAB_INTAKED_BLOCK_WITH_LATCHES = (robot) -> {
        SET_FOUNDATION_LATCHES_DOWN.runOnce(robot);
        GRAB_INTAKED_BLOCK.runOnce(robot);
    };
}