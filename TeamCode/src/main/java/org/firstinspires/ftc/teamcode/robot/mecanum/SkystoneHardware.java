package org.firstinspires.ftc.teamcode.robot.mecanum;


import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.autonomous.odometry.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.DelayedSubroutine;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Subroutines;
import org.firstinspires.ftc.teamcode.common.AxesSigns;
import org.firstinspires.ftc.teamcode.common.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.common.LoadTimer;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.HorizontalSlide;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.IntakeCurrentQueue;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.common.math.TimePose;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.IntakeCurrent;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.ServoToggle;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.SimpleLift;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

@Config
public class SkystoneHardware {

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
    public ExpansionHubEx chassisHub;
    public ExpansionHubEx mechanicHub;
    public BNO055IMU imu;

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public List<DcMotorEx> allMotors;
    public List<Servo> allServos;
    public List<DcMotorEx> chassisMotors;

    public DcMotorEx intakeLeft;
    public DcMotorEx intakeRight;
    public DcMotorEx liftLeft;
    public DcMotorEx liftRight;

    public ServoToggle blockGrabber;
    public HorizontalSlide blockFlipper;
    public ServoToggle lateralEncoderWheelLifter;

    public ServoToggle leftFoundationLatch;
    public ServoToggle rightFoundationLatch;
    public ServoToggle leftFang;
    public ServoToggle rightFang;

    public ServoToggle parkingMarker;

    /* Uneditable constants */
    public final static double TRACK_WIDTH = 16.5; // in
    public final static double WHEEL_DIAMETER = 4; // in
    public final static double FIELD_RADIUS = 141 / 2.0; // in

    public final static String VUFORIA_KEY = "ARdpTSz/////AAABmQ7KRGisnUoRnab3MRG7YtwixzwiqRsIjj" +
            "kqY7tkci5tbijyA9KkQWQTxmWXvKii7VZmacpaiTk0dKCy73Q1VngkUCG9cn7OPOHFIzeIWSGQEsR8IfcR7q" +
            "mGEVFaU9PvNyUcHPjWTnV/RD6egsUShXGGWiU/ZvUm2CyIx7O5bxJYuGLha9WsKj0JVkNTaKr/JdKDs/+bEl" +
            "a8V7Se9Eo2C0PTvqjkOlHpiG/4M55j2HgYLJzt3yz9tMgT5620G1pGgdEBHDar00+Pl1f3p0rymswy8bVeFu" +
            "BZgvksqNEeliKHQzboYuDprMp/dkqGIC57A6kYDKGie8XVirBGa07PhhuVtgtywwqxGNVlKFQ5ta5T";

    /* Tunable parameters */
    public static double INTAKE_UNJAM_REVERSAL_TIME_MS = 200;

    public static int TRAY_DETECTOR_PORT = 0;
    public static double HAS_BLOCK_TRAY_THRESHOLD = 150;
    public static int CLAWS_DETECTOR_PORT = 1;
    public static double HAS_BLOCK_CLAWS_THRESHOLD = 150;
    public static int LEFT_LIMIT_SWITCH_PORT = 0;
    public static int RIGHT_LIMIT_SWITCH_PORT = 2;

    /* Servo positions */
    public static double BLOCK_GRABBER_CLOSED = 0;
    public static double BLOCK_GRABBER_OPEN = 0.5;
    public static double BLOCK_GRABBER_CAPSTONE = 1;

    public static double FOUNDATION_LATCH_OPEN = 0.1;
    public static double FOUNDATION_LATCH_CLOSED = 0.74;
    public static double FOUNDATION_LATCH_OUT = 0.55;
    public static double FOUNDATION_LATCH_LR_OFFSET = -0.02;

    public static double FANGS_RAISED = 0.55;
    public static double FANGS_CLOSED = 0.2;
    public static double FANGS_LR_OFFSET = -0.13;


    public static double WHEEL_LIFTER_DOWN = 0.25;
    public static double WHEEL_LIFTER_UP = 0.6;

    public static double PARKING_MARKER_IN = 0.8;
    public static double PARKING_MARKER_OUT = 0.4;

    /**
     * Instantiates a <b>real</b> SkystoneHardware object that will try to communicate with the REV
     * hub. Always requires a start position.
     *
     * @param hardwareMap The hardwareMap from which to read our devices
     * @param start The robot's starting location on the field
     */
    public SkystoneHardware(HardwareMap hardwareMap, Telemetry telemetry, FtcDashboard dashboard, Pose start) {
        LoadTimer loadTime = new LoadTimer();

        /* Copy dashboard */
        this.dashboard = dashboard;

        /* Drive motors */
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        powers = new MecanumPowers(0, 0, 0, 0);
        // Set up fast access lists
        chassisMotors = Arrays.asList(frontLeft, frontRight, backLeft, backRight);

        /* Intake */
        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        lastIntakeCurrent = new IntakeCurrent(0, 0);
        intakeCurrentQueue = new IntakeCurrentQueue();

        /* Lift and block grabbers */
        pidLift = new SimpleLift(liftLeft, liftRight); // Also initializes lift

        allMotors = Arrays.asList(frontLeft, backLeft, frontRight, backRight, intakeLeft, intakeRight, liftLeft, liftRight);

        blockGrabber = new ServoToggle(
                hardwareMap.get(Servo.class, "blockGrabber"),
                BLOCK_GRABBER_OPEN, BLOCK_GRABBER_CLOSED);
        blockFlipper = new HorizontalSlide(
                hardwareMap.get(Servo.class, "leftBlockFlipper"),
                hardwareMap.get(Servo.class, "rightBlockFlipper"));
        lateralEncoderWheelLifter = new ServoToggle(
                hardwareMap.get(Servo.class, "lateralEncoderWheelLifter"),
                WHEEL_LIFTER_DOWN, WHEEL_LIFTER_UP);

        /* Latches */
        leftFoundationLatch = new ServoToggle(
                hardwareMap.get(Servo.class, "leftFoundationLatch"),
                FOUNDATION_LATCH_OPEN + FOUNDATION_LATCH_LR_OFFSET,
                FOUNDATION_LATCH_CLOSED + FOUNDATION_LATCH_LR_OFFSET);
        rightFoundationLatch = new ServoToggle(
                hardwareMap.get(Servo.class, "rightFoundationLatch"),
                FOUNDATION_LATCH_OPEN - FOUNDATION_LATCH_LR_OFFSET,
                FOUNDATION_LATCH_CLOSED - FOUNDATION_LATCH_LR_OFFSET,
                Servo.Direction.REVERSE);

        /* Fangs */
        leftFang = new ServoToggle(
                hardwareMap.get(Servo.class, "leftFang"),
                FANGS_RAISED + FANGS_LR_OFFSET,
                FANGS_CLOSED + FANGS_LR_OFFSET,
                Servo.Direction.REVERSE);
        rightFang = new ServoToggle(
                hardwareMap.get(Servo.class, "rightFang"),
                FANGS_RAISED - FANGS_LR_OFFSET,
                FANGS_CLOSED - FANGS_LR_OFFSET);

        parkingMarker = new ServoToggle(
                hardwareMap.get(Servo.class, "parkingMarker"),
                PARKING_MARKER_IN,
                PARKING_MARKER_OUT
        );

        allServos = Arrays.asList(blockFlipper.leftFlipper, blockFlipper.rightFlipper,
                leftFoundationLatch.servo, rightFoundationLatch.servo, blockGrabber.servo, leftFang.servo, rightFang.servo);

        /* Hubs for bulk reads */
        chassisHub = hardwareMap.get(ExpansionHubEx.class, "chassisHub");
        mechanicHub = hardwareMap.get(ExpansionHubEx.class, "mechanicHub");
        lastChassisRead = null;

        /* Perform calibration */
        LoadTimer calTime = new LoadTimer();
        initBNO055IMU(hardwareMap);
        for (DcMotorEx m : chassisMotors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        calTime.stop();

        /* Init localization */
        TimePose startPose4D = new TimePose(start, System.currentTimeMillis());
        localizer = new TwoWheelTrackingLocalizer(0, 1, startPose4D);
        this.lastHeading = 0;

        /* Action cache */
        actionCache = new LinkedList<>();

        /* Telemetry */
        this.telemetry = telemetry;
        initTelemetry();
        logBootTelemetry(hardwareMap, loadTime, calTime);
    }

    public SkystoneHardware() {} // Used for debugging

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

    private void initBNO055IMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled  = false;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        headingOffset = imu.getAngularOrientation().firstAngle;
    }

    public void initBulkReadTelemetry() {
        Telemetry.Line odometryLine = telemetry.addLine();
        telOdometry = new Telemetry.Item[3];
        telOdometry[0] = odometryLine.addData("X", "0");
        telOdometry[1] = odometryLine.addData("Y", "0");
        telOdometry[2] = odometryLine.addData("θ", "0");

        Telemetry.Line encoderLine = telemetry.addLine();
        telEncoders = new Telemetry.Item[4];
        for (int i = 0; i < 4; i++) {
            telEncoders[i] = encoderLine.addData("E" + i, -1);
        }

        Telemetry.Line powersLine = telemetry.addLine();
        telPowers = new Telemetry.Item[4];
        telPowers[0] = powersLine.addData("FL", "0");
        telPowers[1] = powersLine.addData("FR", "0");
        telPowers[2] = powersLine.addData("BL", "0");
        telPowers[3] = powersLine.addData("BR", "0");

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
                mechanicHub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS, 0),
                mechanicHub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS, 1)
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
        packet.put("e0", lastChassisRead.getMotorCurrentPosition(0));
        packet.put("e1", lastChassisRead.getMotorCurrentPosition(1));

        packet.fieldOverlay()
                .setFill("blue")
                .fillCircle(localizer.x(), localizer.y(), 3);

        /* Update any FtcDashboard parameters */
        blockGrabber.retractPosition = BLOCK_GRABBER_OPEN;
        blockGrabber.extendPosition = BLOCK_GRABBER_CLOSED;
        lateralEncoderWheelLifter.retractPosition = WHEEL_LIFTER_DOWN;
        lateralEncoderWheelLifter.extendPosition = WHEEL_LIFTER_UP;
        leftFoundationLatch.retractPosition = FOUNDATION_LATCH_OPEN + FOUNDATION_LATCH_LR_OFFSET;
        leftFoundationLatch.extendPosition = FOUNDATION_LATCH_CLOSED + FOUNDATION_LATCH_LR_OFFSET;
        rightFoundationLatch.retractPosition = FOUNDATION_LATCH_OPEN - FOUNDATION_LATCH_LR_OFFSET;
        rightFoundationLatch.extendPosition = FOUNDATION_LATCH_CLOSED - FOUNDATION_LATCH_LR_OFFSET;
        leftFang.retractPosition = FANGS_RAISED + FANGS_LR_OFFSET;
        leftFang.extendPosition = FANGS_CLOSED + FANGS_LR_OFFSET;
        rightFang.retractPosition = FANGS_RAISED - FANGS_LR_OFFSET;
        rightFang.extendPosition = FANGS_CLOSED - FANGS_LR_OFFSET;
        parkingMarker.retractPosition = PARKING_MARKER_IN;
        parkingMarker.extendPosition = PARKING_MARKER_OUT;

        lastTelemetryUpdate = System.nanoTime();
        return lastChassisRead;
    }

    // FTC Dashboard telemetry functions
    public void drawDashboardPath(PurePursuitPath path) {path.draw(packet.fieldOverlay());}
    public void sendDashboardTelemetryPacket() {
        if (dashboard != null) {
            dashboard.sendTelemetryPacket(packet);
        }
    }

    public boolean hasBlockInTray() {
        return lastChassisRead.getAnalogInputValue(TRAY_DETECTOR_PORT) < HAS_BLOCK_TRAY_THRESHOLD;
    }

    public boolean hasBlockInClaws() {
        return lastChassisRead.getAnalogInputValue(CLAWS_DETECTOR_PORT) < HAS_BLOCK_CLAWS_THRESHOLD;
    }

    public boolean leftLimitSwitchPressed() {
        return lastChassisRead.getDigitalInputState(LEFT_LIMIT_SWITCH_PORT);
    }

    public boolean rightLimitSwitchPressed() {
        return lastChassisRead.getDigitalInputState(RIGHT_LIMIT_SWITCH_PORT);
    }

    public boolean hasAction(String tag) {
        for (DelayedSubroutine s : actionCache) {
            if (tag.equals(s.tag)) {
                return true;
            }
        }
        return false;
    }

    public void setPowers(MecanumPowers powers) {
        this.powers = powers;
        frontLeft.setPower(powers.frontLeft);
        frontRight.setPower(powers.frontRight);
        backLeft.setPower(powers.backLeft);
        backRight.setPower(powers.backRight);
    }

    public void setIntakePower(double d) {
        intakeLeft.setPower(d);
        intakeRight.setPower(d);
    }
}
