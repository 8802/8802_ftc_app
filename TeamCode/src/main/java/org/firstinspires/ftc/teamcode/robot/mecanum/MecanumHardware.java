package org.firstinspires.ftc.teamcode.robot.mecanum;


import android.os.Build;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.autonomous.odometry.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.autonomous.odometry.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.common.LoadTimer;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.common.math.TimePose;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.List;

public class MecanumHardware {

    public Telemetry telemetry;

    private Telemetry.Item[] telOdometry;
    private Telemetry.Item[] telEncoders;
    private Telemetry.Item[] telPowers;
    private Telemetry.Item[] telAnalog;

    private Telemetry.Item telDigital;
    private Telemetry.Item telLoopTime;
    private Telemetry.Item telHertz;
    private long lastTelemetryUpdate;

    private LynxModule chassisLynxHub;
    private ExpansionHubEx chassisHub;
    public BNO055IMU imu;
    private double headingOffset;

    TwoWheelTrackingLocalizer localizer;

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx intakeLeft;
    public DcMotorEx intakeRight;

    private MecanumPowers powers;
    public List<DcMotorEx> chassisMotors;
    public List<DcMotorEx> leftChassisMotors;
    public List<DcMotorEx> rightChassisMotors;

    // Constants, stored in inches
    public static double TRACK_WIDTH = 16.5;
    public static double WHEEL_DIAMETER = 4;
    public static double FIELD_RADIUS = 141 / 2.0;

    public MecanumHardware(OpMode opMode, Pose start) {
        LoadTimer loadTime = new LoadTimer();
        RevExtensions2.init();

        frontLeft = opMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = opMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = opMode.hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = opMode.hardwareMap.get(DcMotorEx.class, "rightBack");

        intakeLeft = opMode.hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = opMode.hardwareMap.get(DcMotorEx.class, "intakeRight");

        chassisLynxHub = opMode.hardwareMap.get(LynxModule.class, "chassisHub");
        chassisHub = opMode.hardwareMap.get(ExpansionHubEx.class, "chassisHub");

        // Reverse left hand motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set up fast access linked lists
        chassisMotors = Arrays.asList(frontLeft, frontRight, backLeft, backRight);
        leftChassisMotors = Arrays.asList(frontLeft, backLeft);
        rightChassisMotors = Arrays.asList(frontRight, backRight);

        // Perform calibration
        LoadTimer calTime = new LoadTimer();
        for (DcMotorEx m : chassisMotors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        calTime.stop();

        // Set up localization with motor names the wheels are connected to
        TimePose startPose4D = new TimePose(start, System.currentTimeMillis());
        localizer = new TwoWheelTrackingLocalizer(0, 1, startPose4D);
        this.powers = new MecanumPowers(0, 0, 0, 0);

        // Set up telemetry
        this.telemetry = opMode.telemetry;
        initTelemetry();
        logBootTelemetry(opMode.hardwareMap, loadTime, calTime);
    }

    public MecanumHardware() {} // Used for debugging

    public Pose pose() {
        return localizer.pose();
    }

    private void initTelemetry() {
        telemetry.setMsTransmissionInterval(50); // Update at 20 Hz
        telemetry.setAutoClear(false); // Force not to autoclear
        telemetry.setItemSeparator("; ");
        telemetry.setCaptionValueSeparator(" ");
    }

    private void logBootTelemetry(HardwareMap hardwareMap, LoadTimer lT, LoadTimer cT) {
        Telemetry.Log log = telemetry.log();
        log.clear();
        log.setCapacity(6);

        log.add("-- 8802 RC by Gavin Uberti --");

        // Build information
        Date buildDate = new Date(BuildConfig.TIMESTAMP);
        SimpleDateFormat dateFormat = new SimpleDateFormat("MM/dd HH:mm:ss");
        String javaVersion = System.getProperty("java.runtime.version");
        log.add("Built " + dateFormat.format(buildDate) + " with Java " + javaVersion);

        // Device information
        log.add(Build.MANUFACTURER + " " + Build.MODEL + " running Android " + Build.VERSION.SDK_INT);

        // Chassis information
        String firmware = chassisHub.getFirmwareVersion();
        int rev = chassisHub.getHardwareRevision();
        log.add(this.getClass().getSimpleName() + " with hub " + rev + " " + firmware);

        // Robot information
        List<LynxModule> revHubs = hardwareMap.getAll(LynxModule.class);
        List<DcMotor> motors = hardwareMap.getAll(DcMotor.class);
        List<Servo> servos = hardwareMap.getAll(Servo.class);
        List<DigitalChannel> digital = hardwareMap.getAll(DigitalChannel.class);
        List<AnalogInput> analog = hardwareMap.getAll(AnalogInput.class);
        List<I2cDevice> i2c = hardwareMap.getAll(I2cDevice.class);
        log.add(revHubs.size() + " Hubs; " + motors.size() + " Motors; " + servos.size() +
                " Servos; " + (digital.size() + analog.size() + i2c.size()) + " Sensors");

        lT.stop();

        // Load information
        log.add("Total time " + lT.millis() + " ms; Calibrate time " + cT.millis() + " ms");
        telemetry.update();
        lastTelemetryUpdate = System.nanoTime();
    }

    public void initBNO055IMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        headingOffset = imu.getAngularOrientation().firstAngle;
    }

    public void initBulkReadTelemetry() {
        Telemetry.Line odometryLine = telemetry.addLine();
        telOdometry = new Telemetry.Item[3];
        telOdometry[0] = odometryLine.addData("X", "%.1f", "-1");
        telOdometry[1] = odometryLine.addData("Y", "%.1f", "-1");
        telOdometry[2] = odometryLine.addData("θ", "%.3f", "-1");

        Telemetry.Line encoderLine = telemetry.addLine();
        telEncoders = new Telemetry.Item[4];
        for (int i = 0; i < 4; i++) {
            telEncoders[i] = encoderLine.addData("E" + i, -1);
        }

        Telemetry.Line powersLine = telemetry.addLine();
        telPowers = new Telemetry.Item[4];
        telPowers[0] = powersLine.addData("FL", "%.2f", "-1");
        telPowers[1] = powersLine.addData("FR", "%.2f", "-1");
        telPowers[2] = powersLine.addData("BL", "%.2f", "-1");
        telPowers[3] = powersLine.addData("BR", "%.2f", "-1");

        Telemetry.Line analogLine = telemetry.addLine();
        telAnalog = new Telemetry.Item[4];
        for (int i = 0; i < 4; i++) {
            telAnalog[i] = analogLine.addData("A" + i, -1);
        }

        telDigital = telemetry.addLine().addData("DIGITALS", "0 0 0 0 0 0 0 0");

        Telemetry.Line timingLine = telemetry.addLine("LOOP ");
        telHertz = timingLine.addData("Hertz", -1);
        telLoopTime = timingLine.addData("Millis", -1);

    }

    public RevBulkData performBulkRead() {
        RevBulkData data = chassisHub.getBulkInputData();
        double heading = imu.getAngularOrientation().firstAngle - headingOffset;
        localizer.update(data, heading);

        // Adjust telemetry localizer info
        telOdometry[0].setValue(String.format("%.1f", localizer.x()));
        telOdometry[1].setValue(String.format("%.1f", localizer.y()));
        telOdometry[2].setValue(String.format("%.1f", Math.toDegrees(localizer.h())));

        telPowers[0].setValue(String.format("%.2f", powers.frontLeft));
        telPowers[1].setValue(String.format("%.2f", powers.frontRight));
        telPowers[2].setValue(String.format("%.2f", powers.backLeft));
        telPowers[3].setValue(String.format("%.2f", powers.backRight));

        // Adjust encoders and analog inputs
        for (int i = 0; i < 4; i++) {
            telEncoders[i].setValue(data.getMotorCurrentPosition(i));
            telAnalog[i].setValue(data.getAnalogInputValue(i));
        }

        // Adjust digital inputs
        StringBuilder digitals = new StringBuilder();
        for (int i = 0; i < 8; i++) {
            digitals.append(data.getDigitalInputState(i) ? 1 : 0).append(" ");
        }
        telDigital.setValue(digitals.toString());

        // Adjust elapsed time
        double elapsed = ((System.nanoTime() - lastTelemetryUpdate) / 1000000.0);
        telLoopTime.setValue("%.1f", elapsed);
        telHertz.setValue("%.1f", 1000 / elapsed);

        // Finalize telemetry update
        telemetry.update();
        lastTelemetryUpdate = System.nanoTime();
        return data;
    }

    public void setPowers(MecanumPowers powers) {
        this.powers = powers;
        frontLeft.setPower(powers.frontLeft);
        frontRight.setPower(powers.frontRight);
        backLeft.setPower(powers.backLeft);
        backRight.setPower(powers.backRight);
        System.out.println("Powers: " + powers.toString());
    }

    public void setIntakePower(double d) {
        intakeLeft.setPower(d);
        intakeRight.setPower(d);
    }
}
