
package org.firstinspires.ftc.teamcode.robot.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.openftc.revextensions2.RevBulkData;

@TeleOp(name="Robot centric teleop")
public class MecanumRobotCentric extends MecanumTeleop {
    @Override
    public boolean fieldCentric() {return true;}
}