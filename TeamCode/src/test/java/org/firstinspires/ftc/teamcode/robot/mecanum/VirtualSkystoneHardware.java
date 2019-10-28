package org.firstinspires.ftc.teamcode.robot.mecanum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.simulator.VirtualRobot;
import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.autonomous.odometry.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.common.math.TimePose;
import org.mockito.Mockito;
import org.mockito.stubbing.Answer;
import org.openftc.revextensions2.RevBulkData;

public class VirtualSkystoneHardware extends SkystoneHardware implements VirtualRobot {
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
    boolean slip;
    MecanumPowers wheelPowers;

    public VirtualSkystoneHardware() {
        this(new Pose(0, 0, 0));
    }

    public VirtualSkystoneHardware(Pose position) {
        this(position, true);
    }

    public VirtualSkystoneHardware(Pose position, boolean slip) {
        super(mockHardwareMap(), mockTelemetry(), Mockito.mock(FtcDashboard.class), position);

        this.slip = slip;
        this.position = position;
        this.velocity = new Pose(0, 0, 0);
    }

    private static HardwareMap mockHardwareMap() {
        HardwareMap mockHardwareMap = Mockito.mock(HardwareMap.class);
        Mockito.doAnswer((Answer<HardwareDevice>) invocation -> {
            Object[] args = invocation.getArguments();

            HardwareDevice device = Mockito.mock((Class<HardwareDevice>) args[0]);
            Mockito.doReturn(args[1]).when(device).getDeviceName();

            // A few devices need special functions mocked
            if (device instanceof BNO055IMUImpl) {
                Orientation imuOrientation = new Orientation();
                imuOrientation.firstAngle = 0;
                imuOrientation.secondAngle = 0;
                imuOrientation.thirdAngle = 0;
                Mockito.doReturn(imuOrientation).when((BNO055IMUImpl) device).getAngularOrientation();
            }
            return device;

        }).when(mockHardwareMap).get(Mockito.any(), Mockito.anyString());
        return mockHardwareMap;
    }

    private static Telemetry mockTelemetry() {
        Telemetry mockTelemetry = Mockito.mock(Telemetry.class);
        Mockito.doReturn(Mockito.mock(Telemetry.Log.class)).when(mockTelemetry).log();

        Telemetry.Line mockLine = Mockito.mock(Telemetry.Line.class);
        Mockito.doReturn(Mockito.mock(Telemetry.Item.class)).when(mockLine).addData(Mockito.any(), Mockito.any());
        Mockito.doReturn(mockLine).when(mockTelemetry).addLine();
        Mockito.doReturn(mockLine).when(mockTelemetry).addLine(Mockito.any());

        return mockTelemetry;
    }

    @Override
    public RevBulkData performBulkRead() {
        this.localizer.virtualUpdate(new TimePose(this.position, (long) time * 1000));
        this.packet = new TelemetryPacket();
        System.out.println("Position: " + this.localizer.pose().toString());
        return Mockito.mock(RevBulkData.class);
    }

    @Override
    public void setPowers(MecanumPowers powers) {
        this.wheelPowers = powers;
        System.out.println("Powers: " + powers.toString());
    }

    // Just throw away calls to these functions
    @Override
    public void drawDashboardPath(PurePursuitPath path) {}
    @Override
    public void sendDashboardTelemetryPacket() {}

    // Some method calls we just throw away
    @Override
    public void setIntakePower(double power) {
        System.out.println(String.format("Intake power %.1f", power));
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

        if (slip) {
            velocity = velocity.scale(1 - DECAY_FRAC).add(acceleration.scale(DECAY_FRAC));
            position = MathUtil.relativeOdometryUpdate(position, velocity.scale(secs));
        } else {
            position = MathUtil.relativeOdometryUpdate(position, acceleration.scale(secs));
        }

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
