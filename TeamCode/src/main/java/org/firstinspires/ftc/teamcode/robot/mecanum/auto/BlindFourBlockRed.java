package org.firstinspires.ftc.teamcode.robot.mecanum.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.elements.Alliance;

@Autonomous(name="Four block red", group="4Block")
public class BlindFourBlockRed extends SSAutoLM1 {
    @Override
    public void init() {
        this.ALLIANCE = Alliance.RED;
        super.init();
    }
}
