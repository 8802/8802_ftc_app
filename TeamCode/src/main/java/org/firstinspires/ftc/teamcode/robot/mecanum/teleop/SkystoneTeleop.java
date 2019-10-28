package org.firstinspires.ftc.teamcode.robot.mecanum.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.openftc.revextensions2.RevBulkData;


@Config
public abstract class SkystoneTeleop extends SimulatableMecanumOpMode {
    public static int PER_TICK_LIFT_INCREMENT = 1;

    SkystoneHardware robot;

    boolean leftStickButtonPrev, rightStickButtonPrev, leftBumperPrev, rightBumperPrev, aPrev, yPrev;

    boolean intakeOn;

    // Adjustable properties
    public abstract boolean fieldCentric();

    @Override
    public void init() {
        this.robot = this.getRobot();
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

        intakeOn = false;
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
        } else if (!gamepad1.left_stick_button) {
            leftStickButtonPrev = false;
        }


        if (gamepad1.left_bumper && !leftBumperPrev) {
            leftBumperPrev = true;
            robot.pidLift.changeLayer(-1);
        } else if (!gamepad1.left_bumper) {
            leftBumperPrev = false;
        }

        if (gamepad1.right_bumper && !rightBumperPrev) {
            rightBumperPrev = true;
            robot.pidLift.changeLayer(1);
        } else if (!gamepad1.right_bumper) {
            rightBumperPrev = false;
        }

        if (gamepad1.left_trigger > 0.2) {
            robot.pidLift.changePosition(PER_TICK_LIFT_INCREMENT);
        } else if (gamepad1.right_trigger > 0.2) {
            robot.pidLift.changePosition(-PER_TICK_LIFT_INCREMENT);
        }
    }
}
