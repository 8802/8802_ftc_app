package org.firstinspires.ftc.simulator.utils;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.mockito.Mockito;

public class MockServo implements Servo {
    Direction direction;
    double position;
    public String tag;

    public MockServo (String tag) {
        this.direction = direction.FORWARD;
        this.position = -1;
        this.tag = tag;
    }

    @Override
    public void setPosition(double position) {
        this.position = position;
    }

    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    @Override
    public double getPosition() {
        return position;
    }

    @Override
    public void scaleRange(double min, double max) {

    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

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
