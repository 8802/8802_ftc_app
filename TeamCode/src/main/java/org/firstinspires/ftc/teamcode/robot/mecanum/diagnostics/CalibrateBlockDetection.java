package org.firstinspires.ftc.teamcode.robot.mecanum.diagnostics;

import android.media.AudioManager;
import android.media.ToneGenerator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;
import org.openftc.revextensions2.RevBulkData;

@TeleOp
public class CalibrateBlockDetection extends SimulatableMecanumOpMode {
    SkystoneHardware robot;
    FtcDashboard dashboard;
    ToneGenerator toneGen;
    boolean hadBlock;

    @Override
    public void init() {
        this.dashboard = FtcDashboard.getInstance();
        this.toneGen = new ToneGenerator(AudioManager.STREAM_MUSIC, 100);
        this.robot = this.getRobot();
        this.hadBlock = false;
    }

    @Override
    public void start() {
        robot.initBulkReadTelemetry();
        robot.setIntakePower(1);
    }

    @Override
    public void loop() {
        RevBulkData data = robot.performBulkRead();
        boolean hasBlock = robot.intakeCurrentQueue.hasBlock();
        robot.packet.put("hasBlock", hasBlock);
        if (hasBlock && !hadBlock) {
            toneGen.startTone(ToneGenerator.TONE_PROP_BEEP2, 150);
            hadBlock = true;
        }
        hadBlock = hasBlock;
        robot.sendDashboardTelemetryPacket();
    }
}
