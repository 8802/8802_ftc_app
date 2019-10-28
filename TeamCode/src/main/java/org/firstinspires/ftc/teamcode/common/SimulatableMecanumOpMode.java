package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

public abstract class SimulatableMecanumOpMode extends OpMode {
    // Start in center of field by default (this is of course illegal)
    Pose DEFAULT_START_POSE = new Pose(0, 0, 0);

    public SkystoneHardware getRobot(Pose start) {
        return new SkystoneHardware(this.hardwareMap, this.telemetry, FtcDashboard.getInstance(), start);
    }

    public void stop() {
        requestOpModeStop();
    }

    public SkystoneHardware getRobot() {
        return this.getRobot(DEFAULT_START_POSE);
    }
}
