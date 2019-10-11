package org.firstinspires.ftc.teamcode.robot.mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.simulator.VirtualRobot;
import org.firstinspires.ftc.teamcode.autonomous.odometry.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.autonomous.odometry.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.common.math.TimePose;
import org.mockito.Mockito;
import org.openftc.revextensions2.RevBulkData;

public class VirtualMecanumHardware extends MecanumHardware implements VirtualRobot {
    public static double TRACK_WIDTH = 17;

    public static double MAX_FORWARD_SPEED = 60; // Per second
    public static double MAX_STRAFE_SPEED = 50;

    public static double DECAY_FRAC = 0.1; // How fast speed decays off the robot
    public static Pose MAX_ACCERATIONS = new Pose(MAX_FORWARD_SPEED, MAX_STRAFE_SPEED,
            MAX_FORWARD_SPEED/TRACK_WIDTH);

    public static double MAX_VOLTAGE_ERROR = 0.15; // 1.8 V
    public static double POWER_NONLINEARITY = 1;

    Pose position;
    Pose velocity;
    double time;
    MecanumPowers wheelPowers;

    public VirtualMecanumHardware() {
        this(new Pose(0, 0, 0));
    }

    public VirtualMecanumHardware(Pose position) {
        this.position = position;
        this.velocity = new Pose(0, 0, 0);
        this.time = 0;
        this.localizer = new TwoWheelTrackingLocalizer(0, 2);
        this.wheelPowers = new MecanumPowers(0, 0, 0, 0);
    }

    @Override
    public void initBNO055IMU(HardwareMap hardwareMap) {
        this.imu = Mockito.mock(BNO055IMU.class);
    }

    @Override
    public void initBulkReadTelemetry() {

    }

    @Override
    public RevBulkData performBulkRead() {
        this.localizer.virtualUpdate(new TimePose(this.position, (long) time * 1000));
        System.out.println("Position: " + this.localizer.pose().toString());
        return Mockito.mock(RevBulkData.class);
    }

    @Override
    public void setPowers(MecanumPowers powers) {
        this.wheelPowers = powers;
        System.out.println("Powers: " + powers.toString());
    }

    public Pose pose() {
        return position;
    }

    @Override
    public void elapse(double secs) {
        if (
                Math.abs(wheelPowers.frontLeft) > 1 ||
                Math.abs(wheelPowers.frontRight) > 1 ||
                Math.abs(wheelPowers.backLeft) > 1 ||
                Math.abs(wheelPowers.backRight) > 1
        ) {
            throw new AssertionError();
        }

        MecanumPowers errPowers = new MecanumPowers(
                addVoltageError(wheelPowers.frontLeft),
                addVoltageError(wheelPowers.frontRight),
                addVoltageError(wheelPowers.backLeft),
                addVoltageError(wheelPowers.backRight)
        );

        // +y is to the left
        // Calculations are simple because our wheels are oriented at 45 degrees
        Pose acceleration = new Pose(
                (errPowers.frontLeft +
                    errPowers.frontRight +
                    errPowers.backLeft +
                    errPowers.backRight) / 4,

                (errPowers.frontRight +
                    errPowers.backLeft +
                    -errPowers.frontLeft +
                    -errPowers.backRight) / 4,

            (errPowers.frontRight +
                     errPowers.backRight +
                     -errPowers.frontLeft +
                     -errPowers.backLeft) / 4
        ).multiply(MAX_ACCERATIONS);
        position = MathUtil.relativeOdometryUpdate(position, acceleration.scale(secs));

        //velocity = velocity.scale(1 - DECAY_FRAC).add(acceleration.scale(DECAY_FRAC));
        //position = MathUtil.relativeOdometryUpdate(position, velocity.scale(secs));

        time += secs;
    }

    public static double addVoltageError(double d) {
        double randomFrac = 0;//(Math.random() * 2) - 1; // Between -1 and 1
        double nonlinear = Math.copySign(Math.pow(Math.abs(d), POWER_NONLINEARITY), d);
        return Range.clip(nonlinear + randomFrac, -1, 1);
    }

    @Override
    public Pose getStampedPosition() {
        return new Pose(position.x, position.y, position.heading);
    }
}
