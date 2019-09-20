
package org.firstinspires.ftc.teamcode.robot.mecanum.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot centric teleop")
public class MecanumRobotCentric extends MecanumTeleop {
    @Override
    public boolean fieldCentric() {return false;}
}