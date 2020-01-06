package org.firstinspires.ftc.teamcode.robot.mecanum.diagnostics;

import android.media.AudioManager;
import android.media.ToneGenerator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.DoubleMotorLift;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.IntakeCurrent;
import org.openftc.revextensions2.ExpansionHubEx;

@TeleOp
@Config
public class CalibrateLiftPID extends LinearOpMode {
    FtcDashboard dashboard;
    DoubleMotorLift doublePIDLift;

    public static int target = 0;

    @Override
    public void runOpMode() {
        this.dashboard = FtcDashboard.getInstance();
        this.doublePIDLift = new DoubleMotorLift(
                hardwareMap.get(DcMotorEx.class, "liftLeft"),
                hardwareMap.get(DcMotorEx.class, "liftRight"));
        waitForStart();

        while(opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            doublePIDLift.setTarget(target);
            packet.put("power", doublePIDLift.update());

            packet.put("error", doublePIDLift.prev_error);
            packet.put("integral", doublePIDLift.integral);
            packet.put("derivative", doublePIDLift.derivative);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
