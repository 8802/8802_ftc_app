package org.firstinspires.ftc.teamcode.robot.mecanum.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.elements.Alliance;

@Autonomous(name="Red five block", group="5Block")
public class BlindFourBlockRed extends SSAutoMovingFoundationRed {
    @Override
    public void init() {
        this.ALLIANCE = Alliance.RED;
        super.init();
    }
}
