
package org.firstinspires.ftc.teamcode.robot.mecanum.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Middle pegs teleop")
public class MiddlePegsTeleop extends SkystoneTeleop {
    @Override
    public boolean frontPegs() {
        return false;
    }
}