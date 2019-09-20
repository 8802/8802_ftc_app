
package org.firstinspires.ftc.teamcode.robot.mecanum.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Field centric teleop")
public class MecanumFieldCentric extends MecanumTeleop {
    @Override
    public boolean fieldCentric() {return true;}
}