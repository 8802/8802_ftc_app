package org.firstinspires.ftc.teamcode.robot.mecanum;


import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.autonomous.odometry.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.DelayedSubroutine;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Subroutines;
import org.firstinspires.ftc.teamcode.common.LoadTimer;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.IntakeCurrentQueue;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.common.math.TimePose;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.IntakeCurrent;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.SimpleLift;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

@Config
public class MecanumHardware {

    /* Telemetry */
    public Telemetry telemetry;
    private FtcDashboard dashboard;
    public TelemetryPacket packet;

    private Telemetry.Item[] telOdometry;
    private Telemetry.Item[] telEncoders;
    private Telemetry.Item[] telPowers;
    private Telemetry.Item[] telAnalog;

    private Telemetry.Item telDigital;
    private Telemetry.Item telLoopTime;
    private Telemetry.Item telHertz;
    private long lastTelemetryUpdate;

    /* Internal state */
    /* Odometry */
    private double headingOffset;
    public TwoWheelTrackingLocalizer localizer;

    /* Action cache */
    public LinkedList<DelayedSubroutine> actionCache;

    /* Misc. state */
    public double lastHeading;
    public IntakeCurrent lastIntakeCurrent;
    public IntakeCurrentQueue intakeCurrentQueue;
    public SimpleLift pidLift;

    public RevBulkData lastChassisRead;
    private MecanumPowers powers;

    /* Components */
    private ExpansionHubEx chassisHub;
    private ExpansionHubEx mechanicHub;
    public BNO055IMU imu;
    public WebcamName camera;

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx intakeLeft;
    public DcMotorEx intakeRight;
    public DcMotorEx lift;

    public List<DcMotorEx> chassisMotors;
    public List<DcMotorEx> leftChassisMotors;
    public List<DcMotorEx> rightChassisMotors;

    /* Constants */
    public static double TRACK_WIDTH = 16.5; // in
    public static double WHEEL_DIAMETER = 4; // in
    public static double FIELD_RADIUS = 141 / 2.0; // in

    public static final String VUFORIA_KEY = "ARdpTSz/////AAABmQ7KRGisnUoRnab3MRG7YtwixzwiqRsIjj" +
            "kqY7tkci5tbijyA9KkQWQTxmWXvKii7VZmacpaiTk0dKCy73Q1VngkUCG9cn7OPOHFIzeIWSGQEsR8IfcR7q" +
            "mGEVFaU9PvNyUcHPjWTnV/RD6egsUShXGGWiU/ZvUm2CyIx7O5bxJYuGLha9WsKj0JVkNTaKr/JdKDs/+bEl" +
            "a8V7Se9Eo2C0PTvqjkOlHpiG/4M55j2HgYLJzt3yz9tMgT5620G1pGgdEBHDar00+Pl1f3p0rymswy8bVeFu" +
            "BZgvksqNEeliKHQzboYuDprMp/dkqGIC57A6kYDKGie8XVirBGa07PhhuVtgtywwqxGNVlKFQ5ta5T";

    public static double INTAKE_UNJAM_REVERSAL_TIME_MS = 200;

    public MecanumHardware(OpMode opMode, Pose start) {
        LoadTimer loadTime = new LoadTimer();
        this.dashboard = FtcDashboard.getInstance();
        RevExtensions2.init();

        frontLeft = opMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = opMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = opMode.hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = opMode.hardwareMap.get(DcMotorEx.class, "rightBack");

        intakeLeft = opMode.hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = opMode.hardwareMap.get(DcMotorEx.class, "intakeRight");
        lift = opMode.hardwareMap.get(DcMotorEx.class, "lift");

        chassisHub = opMode.hardwareMap.get(ExpansionHubEx.class, "chassisHub");
        mechanicHub = opMode.hardwareMap.get(ExpansionHubEx.class, "mechanicHub");

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
        this.lastHeading = 0;
        this.lastIntakeCurrent = new IntakeCurrent(0, 0);
        this.intakeCurrentQueue = new IntakeCurrentQueue();
        this.pidLift = new SimpleLift(lift); // Also initializes lift

        this.lastChassisRead = null;

        // Set up action cache
        actionCache = new LinkedList<>();

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

    public void initCamera(HardwareMap hardwareMap) {
        camera = hardwareMap.get(WebcamName.class, "webcam");
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
        this.lastChassisRead = chassisHub.getBulkInputData();
        this.lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        localizer.update(lastChassisRead, lastHeading);

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
            telEncoders[i].setValue(lastChassisRead.getMotorCurrentPosition(i));
            telAnalog[i].setValue(lastChassisRead.getAnalogInputValue(i));
        }

        // Adjust digital inputs
        StringBuilder digitals = new StringBuilder();
        for (int i = 0; i < 8; i++) {
            digitals.append(lastChassisRead.getDigitalInputState(i) ? 1 : 0).append(" ");
        }
        telDigital.setValue(digitals.toString());

        // Adjust motor current and specialty reads
        // We need to read both motors for current to check if they both spike simultaneously
        this.lastIntakeCurrent = new IntakeCurrent(
                mechanicHub.getMotorCurrentDraw(0),
                mechanicHub.getMotorCurrentDraw(1)
        );
        this.intakeCurrentQueue.add(this.lastIntakeCurrent);

        // If we're stalling the intake, reverse for 200 milliseconds
        if (intakeCurrentQueue.stalled()) {
            this.setIntakePower(-1);
            this.actionCache.add(new DelayedSubroutine((long) INTAKE_UNJAM_REVERSAL_TIME_MS, Subroutines.ENABLE_INTAKE));
        }

        // Run any cached actions
        Iterator<DelayedSubroutine> iterator = actionCache.listIterator();
        long timeMillis = System.currentTimeMillis();
        while(iterator.hasNext()) {
            DelayedSubroutine action = iterator.next();
            if (action.systemActionTime < timeMillis) {
                action.action.runOnce(this);
                iterator.remove();
            }
        }

        // Adjust elapsed time
        double elapsed = ((System.nanoTime() - lastTelemetryUpdate) / 1000000.0);
        telLoopTime.setValue("%.1f", elapsed);
        telHertz.setValue("%.1f", 1000 / elapsed);

        // Finalize telemetry update
        telemetry.update();

        this.packet = new TelemetryPacket();
        packet.put("x", localizer.x());
        packet.put("y", localizer.y());
        packet.put("h", localizer.h());
        packet.put("leftMAmps", lastIntakeCurrent.leftMAmps);
        packet.put("rightMAmps", lastIntakeCurrent.rightMAmps);
        packet.fieldOverlay()
                .setFill("blue")
                .fillCircle(localizer.x(), localizer.y(), 3);

        lastTelemetryUpdate = System.nanoTime();
        return lastChassisRead;
    }

    // FTC Dashboard telemetry functions
    public void drawDashboardPath(PurePursuitPath path) {path.draw(packet.fieldOverlay());}
    public void sendDashboardTelemetryPacket() {
        dashboard.sendTelemetryPacket(packet);
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
