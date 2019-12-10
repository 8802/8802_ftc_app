package org.firstinspires.ftc.teamcode.robot.mecanum.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.odometry.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.ServoToggle;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/* Self test is a routine to verify each of the robot's components is working correctly. It should
be run prior to every competition match, preferably multiple times (once while there's still time to
fix things that are broken, once directly before placing it on the field.

List of checks self-test should perform:


 */

@Config
public abstract class SelfTest extends LinearOpMode {

    public static double MONITOR_12V_MIN_VOLTAGE = 12; // Volts
    public static double MONITOR_5V_MIN_VOLTAGE = 5; // Volts

    public static double MOTOR_MAX_DISABLED_POWER = 0.1; // Amps

    public static double DRIVE_MOTOR_RUNNING_MIN_POWER = 1; // Amps
    public static double DRIVE_MOTOR_RUNNING_MAX_POWER = 4; // Amps
    public static double DRIVE_MOTOR_DIRECTION_CURRENT_DIFFERENCE = 1; // Amps

    public static int ODOMETRY_MIN_FORWARD_TICKS = 1000;
    public static int ODOMETRY_MIN_SIDEWAYS_TICKS = 1000;
    public static int ODOMETRY_MAX_LR_SLIPPAGE_TICKS = 200;

    public static double INTAKE_RUNNING_MIN_POWER = 1; // Amps
    public static double INTAKE_RUNNING_MAX_POWER = 8; // Amps

    public static int MIN_LIFT_MAX_HEIGHT_ENCODER_TICKS = 3600;
    public static int MAX_LIFT_ZERO_ENCODER_TICKS = 50;
    public static int MIN_LIFT_STALL_POWER = 4; // Amps, note motor will not be operating at full power

    public static double MAX_HOLDING_SERVO_AMPS = 0.5;
    public static double MIN_HOLDING_SERVO_AMPS = 0.1;
    public static int CURRENT_READ_ITERATIONS = 30;


    SkystoneHardware robot;
    Telemetry.Log log;
    List<String> errors;
    // Adjustable properties

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new SkystoneHardware(this.hardwareMap, this.telemetry,
                FtcDashboard.getInstance(), new Pose(0, 0, 0));

        // We don't initialize bulk read telemetry because we'll do our own
        telemetry.clearAll();
        log = telemetry.log();
        log.setCapacity(1024);
        log.setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        log.add("================================");
        log.add("       SKYSTONE self test       ");
        log.add("      Place robot ON GROUND     ");
        log.add("================================");

        // Disable all servos and motors
        for (DcMotorEx m : robot.allMotors) {
            m.setMotorDisable();
        }
        for (Servo s : robot.allServos) {
            s.getController().pwmDisable();
        }

        // We start out with no errors
        errors = new LinkedList<>();

        // Set LEDs to white
        robot.chassisHub.setLedColor(255, 255, 255);
        robot.mechanicHub.setLedColor(255, 255, 255);

        waitForStart();

        // Verify modules work properly while no power is flowing
        verifyHubOperational(robot.chassisHub);
        verifyHubOperational(robot.mechanicHub);

        // Drive motor forward and verify all motors draw right amount of current
        verifyDriveForwardWorks();

        // Intake motors are plugged in
        verifyIntakeWorks(robot.intakeLeft, "intakeLeft");
        verifyIntakeWorks(robot.intakeRight, "intakeRight");

        // Lift motors are plugged in and encoder works
        verifyLiftWorks();

        // Verify each servo motor works

    }

    private void addResult(String message, boolean good) {
        String output = (good ? "OK" : "ERROR") + " - " + message;
        if (!good) {
            errors.add(output);
        }
        log.add(output);
    }

    private void averageHubTotalCurrent(ExpansionHubEx hub) {

    }

    private void verifyServoWorks(ServoToggle s, ExpansionHubEx hub) throws InterruptedException {
        // The problem here
        for (int i = 0; i < )
        double prevAmps = hub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
        s.servo.getController().pwmEnable();
        s.retract();
        wait(100);
        s.extend();

        s.servo.getController().pwmDisable();
    }

    private void verifyLiftWorks() throws InterruptedException {
        robot.lift.setMotorEnable();
        robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.setPower(0);
        wait(250);
        double current = getCurrent(robot.lift);
        int currentMA = (int) current * 1000;
        addResult("Depowered lift is drawing " + currentMA + " mA",
                current < MOTOR_MAX_DISABLED_POWER);

        robot.lift.setPower(0.75);
        wait(5000);

        int position = robot.lift.getCurrentPosition();
        addResult("Lift max up position is " + position + " ticks",
                position > MIN_LIFT_MAX_HEIGHT_ENCODER_TICKS);
        current = getCurrent(robot.lift);
        addResult("Stalling lift is drawing " + current + " A",
                current > MIN_LIFT_STALL_POWER);

        robot.lift.setPower(-1);
        wait(2000);

        position = robot.lift.getCurrentPosition();
        addResult("Lift max down position is " + position + " ticks",
                position < MAX_LIFT_ZERO_ENCODER_TICKS);

        robot.lift.setPower(0);
        robot.lift.setMotorDisable();
    }

    private void verifyIntakeWorks(DcMotorEx m, String deviceName) throws InterruptedException {
        m.setMotorEnable();
        double current = getCurrent(m);
        int currentMA = (int) current * 1000;
        addResult("Depowered intake " + deviceName + " is drawing " + currentMA + " mA",
                current < MOTOR_MAX_DISABLED_POWER);

        m.setPower(1);
        wait(500);
        current = getCurrent(m);
        addResult("Running intake " + deviceName + " is drawing " + current + " A",
                current > INTAKE_RUNNING_MIN_POWER && current < INTAKE_RUNNING_MAX_POWER);

        m.setPower(0);
        wait(500);
        m.setMotorDisable();
    }

    private void verifyDriveForwardWorks() throws InterruptedException {
        String[] names = {"frontLeft", "backLeft", "frontRight", "backRight"};

        for (DcMotorEx m : robot.chassisMotors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setPower(0);
            m.setMotorEnable();
        }

        // For each motor, verify it's not drawing current
        for (int i = 0; i < 4; i++) {
            DcMotorEx m = robot.chassisMotors.get(i);
            double current = getCurrent(m);
            int currentMA = (int) current * 1000;
            addResult("Depowered motor " + names[i] + " is drawing " + currentMA + " mA",
                    current < MOTOR_MAX_DISABLED_POWER);
        }

        // Now, power each motor 1 by one to ensure they're not caught
        for (int i = 0; i < 4; i++) {
            DcMotorEx m = robot.chassisMotors.get(i);
            m.setPower(1);
            wait(750);
            double forwardCurrent = getCurrent(m);
            addResult(names[i] + " running forward draws " + forwardCurrent + " A",
                    forwardCurrent < DRIVE_MOTOR_RUNNING_MAX_POWER && forwardCurrent > DRIVE_MOTOR_RUNNING_MIN_POWER);
            m.setPower(-1);
            wait(750);
            double backwardCurrent = getCurrent(m);
            addResult(names[i] + " running backward draws " + backwardCurrent + " A",
                    backwardCurrent < DRIVE_MOTOR_RUNNING_MAX_POWER &&
                            backwardCurrent > DRIVE_MOTOR_RUNNING_MIN_POWER &&
                            Math.abs(forwardCurrent - backwardCurrent) < DRIVE_MOTOR_DIRECTION_CURRENT_DIFFERENCE);
            m.setPower(0);
            m.setMotorDisable();
            wait(250);
        }

        robot.localizer.currentPosition = new Pose(0, 0, 0);
        robot.localizer.relativeRobotMovement = new Pose(0, 0, 0);

        // Now drive the robot forward and left to ensure encoder wheels work
        for (DcMotorEx m : robot.chassisMotors) {
            m.setMotorEnable();
        }

        RevBulkData startEncoderPositions = robot.chassisHub.getBulkInputData();
        int startForwardEncoderTicks = startEncoderPositions.getMotorCurrentPosition(TwoWheelTrackingLocalizer.PARALLEL_ENCODER_PORT);
        int startLateralEncoderTicks = startEncoderPositions.getMotorCurrentPosition(TwoWheelTrackingLocalizer.LATERAL_ENCODER_PORT);
        robot.setPowers(new MecanumPowers(0.3, 0, 0));
        wait(1000);

        RevBulkData forwardEncoderPositions = robot.chassisHub.getBulkInputData();
        int forwardEncoderTicks = forwardEncoderPositions.getMotorCurrentPosition(TwoWheelTrackingLocalizer.PARALLEL_ENCODER_PORT);
        int lateralEncoderTicks = forwardEncoderPositions.getMotorCurrentPosition(TwoWheelTrackingLocalizer.LATERAL_ENCODER_PORT);

        addResult("Robot moved forward " + (forwardEncoderTicks - startForwardEncoderTicks) + " ticks",
                (forwardEncoderTicks - startForwardEncoderTicks) > ODOMETRY_MIN_FORWARD_TICKS);
        addResult("Robot slipped sideways " + (lateralEncoderTicks - startLateralEncoderTicks) + " ticks",
                (lateralEncoderTicks - startLateralEncoderTicks) < ODOMETRY_MAX_LR_SLIPPAGE_TICKS);

        robot.setPowers(new MecanumPowers(0, 0.3, 0));
        wait(1000);
        RevBulkData endEncoderPositions = robot.chassisHub.getBulkInputData();
        lateralEncoderTicks = endEncoderPositions.getMotorCurrentPosition(TwoWheelTrackingLocalizer.LATERAL_ENCODER_PORT);
        addResult("Robot moved sideways " + (lateralEncoderTicks - startLateralEncoderTicks) + " ticks",
                (lateralEncoderTicks - startLateralEncoderTicks) > ODOMETRY_MIN_SIDEWAYS_TICKS);

        robot.setPowers(MecanumUtil.STOP);
        for (DcMotorEx m : robot.chassisMotors) {
            m.setMotorDisable();
        }
    }

    private void verifyHubOperational(ExpansionHubEx m) {
        String object = "REV module " + m.getDeviceName();

        // Check if REV module is over temp
        double temperature = m.getInternalTemperature(ExpansionHubEx.TemperatureUnits.FAHRENHEIT);
        addResult(object + " is at temperature " + temperature + " F", m.isModuleOverTemp());

        // Check if motor bridges over temp
        for (int i = 0; i < 4; i++) {
            addResult(object + " motor bridge " + i + " temperature", m.isMotorBridgeOverTemp(i));
        }

        // Verify our 12V and 5V ports have 12V and 5V, respectively
        double bus5V = m.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
        double bus12V = m.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS);
        addResult(object + " 5V monitor at voltage " + bus5V, bus5V >= MONITOR_5V_MIN_VOLTAGE);
        addResult(object + " 12V monitor at voltage " + bus12V, bus12V >= MONITOR_12V_MIN_VOLTAGE);
    }

    private double getCurrent(DcMotor motor) {
        int port = motor.getPortNumber();
        if (robot.chassisMotors.contains(motor)) {
            return robot.chassisHub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS, port);
        } else {
            return robot.mechanicHub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS, port);
        }
    }
}
