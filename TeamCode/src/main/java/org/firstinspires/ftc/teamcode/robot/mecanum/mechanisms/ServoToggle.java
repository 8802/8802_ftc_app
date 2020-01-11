package org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ServoToggle {

    public Double retractPosition;
    public Double extendPosition;
    public Servo servo;
    private boolean extended;

    public ServoToggle(Servo servo, Double retractPosition, Double extendPosition, Servo.Direction direction) {
        this.servo = servo;
        servo.setDirection(direction);
        this.retractPosition = retractPosition;
        this.extendPosition = extendPosition;

        servo.setPosition(retractPosition);
    }

    public ServoToggle(Servo servo, Double retractPosition, Double extendPosition) {
        this(servo, retractPosition, extendPosition, Servo.Direction.FORWARD);
    }

    public void toggle() {
        this.extended = !this.extended;
        servo.setPosition(target());
    }

    public void normalize() {
        servo.setPosition(target());
    }

    public void extend() {
        if (!extended) {
            toggle();
        }
    }

    public void retract() {
        if (extended) {
            toggle();
        }
    }

    public boolean extended() {
        return extended;
    }

    private double target() {
        return extended ? extendPosition : retractPosition;
    }
}
