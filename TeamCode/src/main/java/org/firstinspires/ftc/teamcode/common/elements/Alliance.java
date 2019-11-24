package org.firstinspires.ftc.teamcode.common.elements;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public enum Alliance {
    BLUE, RED;

    public RevBlinkinLedDriver.BlinkinPattern getLEDColor() {
        if (this == BLUE) {
            return RevBlinkinLedDriver.BlinkinPattern.BLUE;
        } else {
            return RevBlinkinLedDriver.BlinkinPattern.RED;
        }
    }
}
