package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;

public abstract class SimulatableMecanumOpMode extends OpMode {
    public boolean stopped;

    public MecanumHardware getRobot() {
        return new MecanumHardware(this);
    }
    public Pose getStartingPosition() {return new Pose(0, 0, 0);}
}
