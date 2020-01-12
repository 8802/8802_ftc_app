package org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.math.MathUtil;

@Config
public class DoubleMotorLift {
    public static double K_P = 0.0003;
    public static double K_I = 0;
    public static double K_D = 0.000002;
    public static double G_STATIC = 0.15;

    public static int FULL_THROTTLE_FLOOR = 10000;
    public static int FULL_THROTTLE_CEIL  = 54000;
    public static int BOTTOM_DISABLE_THRESHOLD = 750;
    public static int MAX_POS = 58000;
    public static double SLOW_MAX_SPEED = 0.3;

    public static double MAX_ADJUST_DOWN_POW = -0.2;

    public DcMotor left;
    public DcMotor right;
    ElapsedTime timer;
    double maxPower;

    public int target;
    public int integral;
    public double derivative;
    public int prev_error;
    public double dt;

    public DoubleMotorLift(DcMotor left, DcMotor right) {
        this.left = left;
        this.right = right;

        // Motor will, by default, break at zero power
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.REVERSE);

        timer = new ElapsedTime();
        target = 0;
        integral = 0;
        prev_error = 0;
        dt = 0;
        maxPower = 1;
    }

    public void setTarget(int target) {
        if (target < 0) {
            target = 0;
        } else if (target > MAX_POS) {
            target = MAX_POS;
        }
        this.target = target;
    }

    public void setMaxPower(double power) {
        maxPower = power;
    }

    public double update() {
        // Update time
        dt = timer.seconds();
        timer.reset();

        // X, V, A
        int position = -left.getCurrentPosition(); // Negated because of the gear difference
        int error = target - position;
        integral += error * dt;
        derivative = (error - prev_error) / dt;
        prev_error = error;

        double output = K_P * error + K_I * integral + K_D * derivative + G_STATIC;
        output = Math.max(-maxPower, Math.min(maxPower, output));

        /* If we're in the bottom or top of lift, cut power to prevent destroying the thing */
        if (output > 0 && position > FULL_THROTTLE_CEIL) {
            output = Math.min(SLOW_MAX_SPEED, output);
        } else if (output < 0 && position < FULL_THROTTLE_FLOOR) {
            output = Math.max(-SLOW_MAX_SPEED, output);
        }

        if (target == 0 && Math.abs(error) < BOTTOM_DISABLE_THRESHOLD) {
            output = 0;
        }

        if (target != 0) {
            output = Math.max(MAX_ADJUST_DOWN_POW, output);
        }

        left.setPower(output);
        right.setPower(output);
        return output;
    }
}
