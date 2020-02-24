package org.firstinspires.ftc.simulator.utils;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MockDcMotorEx implements DcMotorEx {
    private double power;
    private int targetPosition;
    private RunMode runMode;
    private boolean enabled;
    private int currentPosition;
    public String tag;

    public MockDcMotorEx(String tag) {
        this.power = 0;
        this.targetPosition = Integer.MIN_VALUE;
        this.runMode = RunMode.RUN_WITHOUT_ENCODER;
        this.enabled = true;
        this.currentPosition = 0;
        this.tag = tag;
    }

    @Override
    public void setMotorEnable() {
        enabled = true;
    }

    @Override
    public void setMotorDisable() {
        enabled = false;
    }

    @Override
    public boolean isMotorEnabled() {
        return enabled;
    }

    /* We don't use any velocity methods, so we don't bother mocking them */
    @Override
    public void setVelocity(double angularRate) {}
    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {}
    @Override
    public double getVelocity() {return 0;}
    @Override
    public double getVelocity(AngleUnit unit) {return 0;}

    /* We don't use PID/PIDF coefficients right now either */
    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {}
    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {}
    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {}
    @Override
    public void setPositionPIDFCoefficients(double p) {}
    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) { return null; }
    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) { return null; }
    @Override
    public void setTargetPositionTolerance(int tolerance) {}
    @Override
    public int getTargetPositionTolerance() { return 0; }
    @Override
    public double getCurrent(CurrentUnit unit) { return 0; }
    @Override
    public double getCurrentAlert(CurrentUnit unit) { return 0; }
    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {}
    @Override
    public boolean isOverCurrent() { return false; }
    @Override
    public MotorConfigurationType getMotorType() { return null; }
    @Override
    public void setMotorType(MotorConfigurationType motorType) {}
    @Override
    public DcMotorController getController() { return null; }
    @Override
    public int getPortNumber() { return 0; }
    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {}
    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() { return null; }
    @Override
    public void setPowerFloat() {}
    @Override
    public boolean getPowerFloat() { return false; }

    @Override
    public void setTargetPosition(int position) {
        this.targetPosition = position;
    }

    @Override
    public int getTargetPosition() {
        return this.targetPosition;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public int getCurrentPosition() {
        return currentPosition;
    }

    public void setCurrentPosition(int position) {
        this.currentPosition = position;
    }

    @Override
    public void setMode(RunMode mode) {
        this.runMode = mode;
    }

    @Override
    public RunMode getMode() {
        return runMode;
    }

    @Override
    public void setDirection(Direction direction) {}
    @Override
    public Direction getDirection() { return null; }

    @Override
    public void setPower(double power) {
        this.power = power;
    }

    @Override
    public double getPower() {
        return power;
    }

    @Override
    public Manufacturer getManufacturer() { return null; }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
