package org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class HorizontalSlide {
    public static double LEFT_MIN = 0.42;
    public static double RIGHT_MIN = 0.42;

    public static double LEFT_MAX = 0.96;
    public static double RIGHT_MAX = 0.96;

    public static double MAX_EXTENSION_IN = 360 / 25.4;

    public static double INTAKING_IN = 0;
    public static double BLOCK_GRAB_IN = 0;
    public static double DRIVING = 4;
    public static double INNER_PEGS = 6;
    public static double MIDDLE_PEGS = 10;
    public static double OUTER_PEGS = 14;

    public Servo leftFlipper;
    public Servo rightFlipper;

    public HorizontalSlide(Servo leftFlipper, Servo rightFlipper) {
        this.leftFlipper = leftFlipper;
        this.rightFlipper = rightFlipper;
        leftFlipper.setDirection(Servo.Direction.REVERSE);
        readyBlockIntake();
    }

    public void readyBlockGrab()   {setPosition(BLOCK_GRAB_IN);}
    public void readyBlockIntake() {setPosition(INTAKING_IN);}
    public void readyDriving()     {setPosition(DRIVING);}
    public void normExtend()       {setPosition(MIDDLE_PEGS);}
    public void maxExtend()        {setPosition(MIDDLE_PEGS);}


    public void setPosition(double inches) {
        leftFlipper.setPosition(LEFT_MIN + (LEFT_MAX - LEFT_MIN) * (inches / MAX_EXTENSION_IN));
        rightFlipper.setPosition(RIGHT_MIN + (RIGHT_MAX - RIGHT_MIN) * (inches / MAX_EXTENSION_IN));
    }


}
