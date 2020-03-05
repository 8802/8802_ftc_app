package org.firstinspires.ftc.teamcode.robot.mecanum.diagnostics;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.ExternalLiftMotor;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.SimpleLift;

@TeleOp
@Config
public class CalibrateLiftPID extends LinearOpMode {
    FtcDashboard dashboard;

    public static int TARGET = 0;
    public static int LAYER = 0;

    public static double V_P = 0.117;
    public static double V_I = 0.017;
    public static double V_D = 0;
    public static double V_F = 1.17;
    public static double P_P = 5;

    @Override
    public void runOpMode() {
        this.dashboard = FtcDashboard.getInstance();
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "liftLeft");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "liftRight");
        SimpleLift pidLift = new SimpleLift(left, right); // Also initializes lift

        double pV_P = V_P;
        double pV_I = V_I;
        double pV_D = V_D;
        double pV_F = V_F;
        double pP_P = P_P;

        /*left.setVelocityPIDFCoefficients(pV_P, pV_I, pV_D, pV_F);
        right.setVelocityPIDFCoefficients(pV_P, pV_I, pV_D, pV_F);
        left.setPositionPIDFCoefficients(pP_P);
        right.setPositionPIDFCoefficients(pP_P);*/

        waitForStart();

        while(opModeIsActive()) {

            if (V_P != pV_P || V_I != pV_I || V_D != pV_D || V_F != pV_F || P_P != pP_P) {
                pV_P = V_P;
                pV_I = V_I;
                pV_D = V_D;
                pV_F = V_F;
                pP_P = P_P;
                /*left.setVelocityPIDFCoefficients(pV_P, pV_I, pV_D, pV_F);
                right.setVelocityPIDFCoefficients(pV_P, pV_I, pV_D, pV_F);
                left.setPositionPIDFCoefficients(pP_P);
                right.setPositionPIDFCoefficients(pP_P);*/
            }

            //left.setTargetPosition(TARGET);
            //right.setTargetPosition(TARGET);
            if (LAYER != pidLift.layer) {
                pidLift.setLayer(LAYER);
            }
            pidLift.update();

            int leftPos = left.getCurrentPosition();
            int rightPos = right.getCurrentPosition();
            int error = TARGET - leftPos;

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("leftPos", leftPos);
            packet.put("rightPos", rightPos);
            packet.put("error", error);
            dashboard.sendTelemetryPacket(packet);

        }
    }
}
