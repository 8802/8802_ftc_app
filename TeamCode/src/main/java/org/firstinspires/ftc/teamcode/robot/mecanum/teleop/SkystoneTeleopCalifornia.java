package org.firstinspires.ftc.teamcode.robot.mecanum.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.autonomous.waypoints.DelayedSubroutine;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Subroutines;
import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.SimpleLift;
import org.openftc.revextensions2.RevBulkData;


@Config
public class SkystoneTeleopCalifornia extends SimulatableMecanumOpMode {
    public static double TRIGGER_THRESHOLD = 0.2;

    SkystoneHardware robot;

    boolean leftStickButtonPrev, rightStickButtonPrev, rightTriggerPrev, leftBumperPrev, rightBumperPrev, aPrev, yPrev, xPrev, bPrev;

    enum RightTriggerActions {
        GRAB, VERIFY, DROP;
        public RightTriggerActions next() {
            switch(this) {
                case GRAB:
                    return VERIFY;
                case VERIFY:
                    return DROP;
                case DROP:
                    return GRAB;
            }
            return null;
        }
    }


    RightTriggerActions nextRightTriggerAction;
    boolean intakeOn;

    // Adjustable properties

    public boolean fieldCentric() {return false;}

    @Override
    public void init() {
        this.robot = this.getRobot();
        robot.pidLift = new SimpleLift(robot.lift, robot.leds);
    }

    @Override
    public void start() {
        robot.initBulkReadTelemetry();

        leftStickButtonPrev = gamepad1.left_stick_button;
        rightStickButtonPrev = gamepad1.right_stick_button;
        leftBumperPrev = gamepad1.left_bumper;
        rightBumperPrev = gamepad1.right_bumper;
        aPrev = gamepad1.a;
        yPrev = gamepad1.y;
        xPrev = gamepad1.x;
        bPrev = gamepad1.b;
        
        intakeOn = false;
        nextRightTriggerAction = RightTriggerActions.GRAB;
    }

    @Override
    public void loop() {
        RevBulkData data = robot.performBulkRead();
        robot.packet.put("layer", robot.pidLift.layer);
        robot.packet.put("targetPosition", robot.pidLift.targetPosition);
        robot.sendDashboardTelemetryPacket();

        /* Drive code */
        double slowScale = ((1 - gamepad1.left_trigger) * 0.7 + 0.3);
        double leftX = MecanumUtil.deadZone(-gamepad1.left_stick_x, 0.05) * slowScale;
        double leftY = MecanumUtil.deadZone(-gamepad1.left_stick_y, 0.05) * slowScale;
        double angle = -Math.atan2(leftY, leftX) + Math.PI / 2;
        if (fieldCentric()) {
            angle -= robot.pose().heading;
        }

        double driveScale = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
        driveScale = Range.clip(driveScale, 0, 1);

        // Exponentiate our turn
        double turn = Math.copySign(
                Math.pow(MecanumUtil.deadZone(-gamepad1.right_stick_x, 0.05), 2),
                -gamepad1.right_stick_x) * slowScale;

        MecanumPowers powers = MecanumUtil.powersFromAngle(angle, driveScale, turn);
        robot.setPowers(powers);

        /* Control intake */

        if (gamepad1.left_stick_button && !leftStickButtonPrev) {
            leftStickButtonPrev = true;
            intakeOn = !intakeOn; // Toggle intake
            robot.setIntakePower(intakeOn ? 1 : 0);
            robot.blockGrabber.retract();
        } else if (!gamepad1.left_stick_button) {
            leftStickButtonPrev = false;
            if (robot.hasBlockInTray() && intakeOn) {
                intakeOn = false;
                robot.setIntakePower(0);
            }
        }

        /* Block grabber */
        if (gamepad1.right_stick_button && !rightStickButtonPrev) {
            rightStickButtonPrev = true;
            /* Do nothing */
        } else if (!gamepad1.right_stick_button) {
            rightStickButtonPrev = false;
        }

        /* Intake flipper */
        boolean leftTrigger = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        boolean rightTrigger = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        if (leftTrigger && !rightTrigger) {
            robot.blockFlipper.readyBlockIntake();
            nextRightTriggerAction = RightTriggerActions.GRAB;
        }

        if (rightTrigger && !rightTriggerPrev) {
            rightTriggerPrev = true;
            switch(nextRightTriggerAction) {
                case GRAB:
                    if (robot.blockGrabber.extended()) {
                        // If we've already grabbed the block, just flip out
                        robot.blockFlipper.readyDriving();
                    } else {
                        robot.blockFlipper.readyBlockGrab();
                        robot.blockGrabber.extend(); // Grab the block
                        robot.actionCache.add(new DelayedSubroutine(600, Subroutines.SET_FLIPPER_DRIVING));
                    }
                    break;

                case VERIFY:
                    robot.blockFlipper.normExtend();
                    break;

                case DROP:
                    robot.blockGrabber.retract();
                    robot.actionCache.add(new DelayedSubroutine(250, Subroutines.LIFT_A_LITTLE));
                    robot.actionCache.add(new DelayedSubroutine(1000, Subroutines.SET_FLIPPER_INTAKING));
                    robot.actionCache.add(new DelayedSubroutine(1000, Subroutines.LIFT_TO_ZERO));
                    break;
            }

            nextRightTriggerAction = nextRightTriggerAction.next();
            rightTriggerPrev = true;
        } else if (!rightTrigger) {
            rightTriggerPrev = false;
        }

        /* Lit control */
        /* Down bumper only goes down on RELEASE */
        if (!gamepad1.left_bumper && leftBumperPrev) {
            leftBumperPrev = false;
            robot.pidLift.changeLayer(-1);
        } else if (gamepad1.left_bumper) {
            leftBumperPrev = true;
        }

        if (gamepad1.right_bumper && !rightBumperPrev) {
            rightBumperPrev = true;
            robot.pidLift.changeLayer(1);
        } else if (!gamepad1.right_bumper) {
            rightBumperPrev = false;
        }

        /* Misc servos */
        if (gamepad1.y && !yPrev) {
            yPrev = true;
            robot.leftFoundationLatch.toggle();
            robot.rightFoundationLatch.toggle();
        } else if (!gamepad1.y) {
            yPrev = false;
        }

        if (gamepad1.a && !aPrev) {
            aPrev = true;
            robot.capstoneDropper.toggle();
        } else if (!gamepad1.a) {
            aPrev = false;
        }

        if (gamepad1.b && !bPrev) {
            bPrev = true;
            if (robot.intakeLeft.getPower() < 0) {
                robot.setIntakePower(0);
            } else {
                robot.setIntakePower(-1);
            }
            intakeOn = false;
        } else if (!gamepad1.b) {
            bPrev = false;
        }

        if (gamepad1.x && !xPrev) {
            xPrev = true;
            robot.blockFlipper.readyBlockIntake();
            robot.blockGrabber.retract();
            nextRightTriggerAction = RightTriggerActions.GRAB;
        } else if (!gamepad1.x) {
            xPrev = false;
        }
    }
}
