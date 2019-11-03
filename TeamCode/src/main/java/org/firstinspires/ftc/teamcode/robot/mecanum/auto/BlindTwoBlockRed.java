package org.firstinspires.ftc.teamcode.robot.mecanum.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.elements.Alliance;

@Autonomous(name="Two block red", group="2Block")
public class BlindTwoBlockRed extends SSAutoMovingFoundationRed {
    @Override
    public void init() {
        this.ALLIANCE = Alliance.RED;
        super.init();
    }
}
