package org.firstinspires.ftc.teamcode.robot.mecanum;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.openftc.revextensions2.RevBulkData;

public abstract class MecanumTeleop extends SimulatableMecanumOpMode {
    MecanumHardware robot;

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
    }

    @Override
    public void loop() {
        RevBulkData data = robot.performBulkRead();
        double leftX = MecanumUtil.deadZone(gamepad1.left_stick_x, 0.05);
        double leftY = MecanumUtil.deadZone(gamepad1.left_stick_y, 0.05);
        double angle = -Math.atan2(leftY, leftX) + Math.PI/2;
        if (fieldCentric()) {angle += robot.pose().heading;}

        double driveScale = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
        driveScale = Range.clip(driveScale, 0, 1);

        // Exponentiate our turn
        double turn = Math.pow(MecanumUtil.deadZone(gamepad1.right_stick_x, 0.05), 3);

        MecanumPowers powers = MecanumUtil.powersFromAngle(angle, driveScale, turn);
        robot.setPowers(powers);
    }
}
