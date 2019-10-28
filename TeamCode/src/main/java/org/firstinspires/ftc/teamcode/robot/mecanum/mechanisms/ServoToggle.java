package org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms;

import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ServoToggle {

    public Double retractPosition;
    public Double extendPosition;
    public ServoImplEx servo;
    private boolean extended;

    public ServoToggle(ServoImplEx servo, Double retractPosition, Double extendPosition) {
        this.servo = servo;
        this.retractPosition = retractPosition;
        this.extendPosition = extendPosition;

        servo.setPosition(retractPosition);
    }

    public void toggle() {
        this.extended = !this.extended;
        servo.setPosition(target());
    }

    private double target() {
        return extended ? extendPosition : retractPosition;
    }
}
