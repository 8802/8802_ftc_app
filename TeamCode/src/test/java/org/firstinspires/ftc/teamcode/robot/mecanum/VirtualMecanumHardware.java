package org.firstinspires.ftc.teamcode.robot.mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.simulator.VirtualRobot;
import org.firstinspires.ftc.teamcode.autonomous.odometry.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.autonomous.odometry.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.sixwheel.SixWheelHardware;
import org.firstinspires.ftc.teamcode.robot.sixwheel.SixWheelPowers;
import org.mockito.Mockito;
import org.openftc.revextensions2.RevBulkData;

public class VirtualMecanumHardware extends MecanumHardware implements VirtualRobot {
    double TRACK_WIDTH = 17;

    double MAX_FORWARD_SPEED = 60; // Per second
    double MAX_STRAFE_SPEED = 50;

    Pose TERMINAL_VELOCITIES = new Pose(MAX_FORWARD_SPEED, MAX_STRAFE_SPEED, MAX_FORWARD_SPEED/TRACK_WIDTH);
    // TODO replace these with better, measured values
    Pose MAX_ACCERATIONS = new Pose(MAX_FORWARD_SPEED, MAX_STRAFE_SPEED, MAX_FORWARD_SPEED/TRACK_WIDTH);

    // At terminal velocity, the robot's frictional acceleration will be equal to the max acceleration,
    // but opposite in direction
    Pose STATIC_FRICTION = new Pose(20, 20, 1);
    Pose KINETIC_FRICTION = TERMINAL_VELOCITIES.clone().scale(-1);


    Pose position;
    Pose velocity;
    double time;
    MecanumPowers wheelPowers;

    public VirtualMecanumHardware(Object o) {
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
        this.localizer.currentPosition = this.position;
        return Mockito.mock(RevBulkData.class);
    }

    @Override
    public void setPowers(MecanumPowers powers) {
        this.wheelPowers = powers;
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

        // +y is to the left
        // Calculations are simple because our wheels are oriented at 45 degrees
        Pose acceleration = new Pose(
                (wheelPowers.frontLeft +
                    wheelPowers.frontRight +
                    wheelPowers.backLeft +
                    wheelPowers.backRight) / 4,

                (wheelPowers.frontRight +
                    wheelPowers.backLeft +
                    -wheelPowers.frontLeft +
                    -wheelPowers.backRight) / 4,

            (wheelPowers.frontRight +
                    wheelPowers.backRight +
                    -wheelPowers.frontLeft +
                    -wheelPowers.backLeft) / 4
        ).multiply(MAX_ACCERATIONS);

        if (false) {
            acceleration.applyFriction(STATIC_FRICTION);

            Pose velocityDelta = acceleration.scale(secs);
            velocity = velocity.add(velocityDelta);
            velocity.clampAbs(TERMINAL_VELOCITIES);
            System.out.println(velocity);
            System.out.println(velocityDelta);
            System.out.println(secs);

            Pose positionDelta = velocity.scale(secs);
            position = MathUtil.relativeOdometryUpdate(position, positionDelta);
        } else {
            position = MathUtil.relativeOdometryUpdate(position, acceleration.scale(secs));
        }
        time += secs;
    }

    @Override
    public Pose getStampedPosition() {
        return new Pose(position.x, position.y, position.heading);
    }
}
