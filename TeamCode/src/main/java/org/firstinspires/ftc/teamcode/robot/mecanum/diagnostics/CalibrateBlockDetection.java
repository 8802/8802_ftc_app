package org.firstinspires.ftc.teamcode.robot.mecanum.diagnostics;

import android.media.AudioManager;
import android.media.ToneGenerator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.waypoints.DelayedSubroutine;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Subroutines;
import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.IntakeCurrent;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.concurrent.Delayed;

@TeleOp
@Disabled
public class CalibrateBlockDetection extends LinearOpMode {
    SkystoneHardware robot;
    FtcDashboard dashboard;
    ToneGenerator toneGen;
    long restartTimeMS;

    @Override
    public void runOpMode() {
        this.dashboard = FtcDashboard.getInstance();
        this.toneGen = new ToneGenerator(AudioManager.STREAM_MUSIC, 100);
        this.robot = new SkystoneHardware(this.hardwareMap, this.telemetry, FtcDashboard.getInstance(), new Pose(0, 0, 0));
        waitForStart();
        robot.setIntakePower(0.5);

        while(opModeIsActive()) {
            robot.intakeCurrentQueue.add(new IntakeCurrent(
                    robot.mechanicHub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS, 0),
                    robot.mechanicHub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS, 1)
            ));
            boolean hasBlock = robot.intakeCurrentQueue.hasBlock();
            if (hasBlock) {
                toneGen.startTone(ToneGenerator.TONE_PROP_BEEP2, 150);
                robot.setIntakePower(0.15);
                sleep(1000);
                robot.setIntakePower(0.5);
                sleep(500);
            }
        }
    }
}
