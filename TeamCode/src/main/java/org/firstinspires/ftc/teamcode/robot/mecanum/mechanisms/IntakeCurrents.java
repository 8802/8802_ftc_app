package org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms;

public class IntakeCurrents {
    public static double GRABBING_BLOCK_THRESHOLD = 4000;

    public double leftCurrent;
    public double rightCurrent;

    public IntakeCurrents(double leftCurrent, double rightCurrent) {
        this.leftCurrent = leftCurrent;
        this.rightCurrent = rightCurrent;
    }

    public double max() {
        return Math.max(leftCurrent, rightCurrent);
    }

    public boolean hasBlock() {
        return this.max() > GRABBING_BLOCK_THRESHOLD;
    }
}
