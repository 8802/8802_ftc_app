package org.firstinspires.ftc.simulator.utils;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.mockito.Mockito;

public class MockServoImplEx extends ServoImplEx {
    Direction direction;
    double position;
    public String tag;

    public MockServoImplEx(String tag) {
        super(Mockito.mock(ServoControllerEx.class), 0, Mockito.mock(ServoConfigurationType.class));
        this.direction = direction.FORWARD;
        this.position = -1;
        this.tag = tag;
    }

    @Override
    public void setPosition(double position) {
        this.position = position;
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
    public Direction getDirection() {
        return direction;
    }
}
