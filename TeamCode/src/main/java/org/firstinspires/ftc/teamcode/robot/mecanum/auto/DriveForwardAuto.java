package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Autonomous(name="Drive forward", group="Utils")
public class DriveForwardAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SkystoneHardware robot = new SkystoneHardware(this.hardwareMap, this.telemetry,
                FtcDashboard.getInstance(), new Pose(0, 0, 0));
        waitForStart();
        robot.setPowers(MecanumUtil.powersFromAngle(0, 0.4, 0));
        sleep(300);
        robot.setPowers(MecanumUtil.STOP);
    }
}
