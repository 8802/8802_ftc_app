package org.firstinspires.ftc.teamcode.robot.mecanum.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.elements.Alliance;

@Autonomous(name="Blue three block", group="3Block")
public class BlindThreeBlockBlue extends SSAutoMovingFoundation {
    @Override
    public void init() {
        this.ALLIANCE = Alliance.BLUE;
        super.init();
    }
}
