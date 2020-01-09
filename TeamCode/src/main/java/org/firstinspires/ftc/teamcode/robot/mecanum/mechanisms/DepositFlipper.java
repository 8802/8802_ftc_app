package org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class DepositFlipper {
    public static double LEFT_GRABBING = 0.31;
    public static double RIGHT_GRABBING = 0.25;

    public static double LEFT_INTAKING = 0.42;
    public static double RIGHT_INTAKING = 0.37;

    public static double LEFT_DRIVING = 0.64;
    public static double RIGHT_DRIVING = 0.6;

    public static double LEFT_NORM_EXTEND = 0.87;
    public static double RIGHT_NORM_EXTEND = 0.81;
    
    public Servo leftFlipper;
    public Servo rightFlipper;

    public DepositFlipper(Servo leftFlipper, Servo rightFlipper) {
        this.leftFlipper = leftFlipper;
        this.rightFlipper = rightFlipper;
        rightFlipper.setDirection(Servo.Direction.REVERSE);
        readyBlockIntake();
    }

    public void readyBlockGrab()   {setPosition(LEFT_GRABBING, RIGHT_GRABBING);}
    public void readyBlockIntake() {setPosition(LEFT_INTAKING, RIGHT_INTAKING);}
    public void readyDriving()     {setPosition(LEFT_DRIVING, RIGHT_DRIVING);}
    public void normExtend()       {setPosition(LEFT_NORM_EXTEND, RIGHT_NORM_EXTEND);}


    public void setPosition(double left, double right) {
        leftFlipper.setPosition(left);
        rightFlipper.setPosition(right);
    }


}
