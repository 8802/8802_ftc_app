package org.firstinspires.ftc.teamcode.robot.mecanum.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.openftc.revextensions2.RevBulkData;


@Config
public abstract class MecanumTeleop extends SimulatableMecanumOpMode {
    public static int PER_TICK_LIFT_INCREMENT = 1;

    MecanumHardware robot;

    boolean dpadUpPrev, dpadDownPrev, leftBumperPrev, rightBumperPrev;
    int intakePower;

    // Adjustable properties
    public abstract boolean fieldCentric();

    @Override
    public void init() {
        this.robot = this.getRobot();
        robot.initBNO055IMU(hardwareMap);
    }

    @Override
    public void start() {
        robot.initBulkReadTelemetry();
        dpadUpPrev = gamepad1.dpad_up;
        dpadDownPrev = gamepad1.dpad_down;
        leftBumperPrev = gamepad1.left_bumper;
        rightBumperPrev = gamepad1.right_bumper;
        intakePower = 0;
    }

    @Override
    public void loop() {
        RevBulkData data = robot.performBulkRead();
        robot.packet.put("layer", robot.pidLift.layer);
        robot.packet.put("targetPosition", robot.pidLift.targetPosition);
        robot.sendDashboardTelemetryPacket();

        // Drive code
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

        // Intake code
        if (gamepad1.dpad_up && !dpadUpPrev) {
            dpadUpPrev = true;
            if (intakePower == 1) {
                intakePower = 0;
            } else {
                intakePower = 1;
            }
            robot.setIntakePower(intakePower);
        } else if (!gamepad1.dpad_up) {
            dpadUpPrev = false;
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
