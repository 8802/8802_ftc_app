package org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class DepositFlipper {
    public static double GRABBING = 0.25;
    public static double INTAKING = 0.35;
    public static double DRIVING = 0.6;
    public static double NORM_EXTEND = 0.8;
    public static double MAX_EXTEND = 0.8;
    public static double LR_OFFSET = 0.007;

    public ServoImplEx leftFlipper;
    public ServoImplEx rightFlipper;

    public DepositFlipper(ServoImplEx leftFlipper, ServoImplEx rightFlipper) {
        this.leftFlipper = leftFlipper;
        this.rightFlipper = rightFlipper;
        rightFlipper.setDirection(Servo.Direction.REVERSE);
    }

    public void readyBlockGrab()   {setPosition(GRABBING);}
    public void readyBlockIntake() {setPosition(INTAKING);}
    public void readyDriving()     {setPosition(DRIVING);}
    public void normExtend()       {setPosition(NORM_EXTEND);}
    public void maxExtend()        {setPosition(MAX_EXTEND);}


    public void setPosition(double position) {
        leftFlipper.setPosition(position + LR_OFFSET);
        rightFlipper.setPosition(position - LR_OFFSET);
    }


}
