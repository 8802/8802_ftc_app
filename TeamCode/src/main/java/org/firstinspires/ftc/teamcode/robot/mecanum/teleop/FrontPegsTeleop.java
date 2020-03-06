
package org.firstinspires.ftc.teamcode.robot.mecanum.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Front pegs teleop")
public class FrontPegsTeleop extends SkystoneTeleop {
    @Override
    public boolean frontPegs() {
        return true;
    }
}