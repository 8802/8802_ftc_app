package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;

public abstract class SimulatableMecanumOpMode extends OpMode {
    // Start in center of field by default (this is of course illegal)
    Pose DEFAULT_START_POSE = new Pose(0, 0, 0);

    public MecanumHardware getRobot(Pose start) {
        return new MecanumHardware(this, start);
    }

    public MecanumHardware getRobot() {
        return this.getRobot(DEFAULT_START_POSE);
    }
}
