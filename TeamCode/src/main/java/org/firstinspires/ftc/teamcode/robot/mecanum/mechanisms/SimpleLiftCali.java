package org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BLACK;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.BLUE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.GOLD;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.GREEN;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RED;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.VIOLET;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.WHITE;

@Config
public class SimpleLiftCali extends SimpleLift {

    public SimpleLiftCali(DcMotorEx lift, RevBlinkinLedDriver leds) {
        super(lift, leds);
    }

    @Override
    public void goToMin() {
        layer = 0;
        setLiftPositionFromLayer();
    }
}
