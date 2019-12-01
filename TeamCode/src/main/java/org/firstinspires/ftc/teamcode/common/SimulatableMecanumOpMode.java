package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.elements.Alliance;
import org.firstinspires.ftc.teamcode.common.elements.SkystoneState;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.auto.vision.ImprovedSkystoneDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public abstract class SimulatableMecanumOpMode extends OpMode {
    // Start in center of field by default (this is of course illegal)
    Pose DEFAULT_START_POSE = new Pose(0, 0, 0);

    // For autonomous
    private OpenCvCamera camera;
    private ImprovedSkystoneDetector detector;

    public SkystoneHardware getRobot(Pose start) {
        return new SkystoneHardware(this.hardwareMap, this.telemetry, FtcDashboard.getInstance(), start);
    }

    public void startPhoneCamDetector(Alliance alliance) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.openCameraDevice();
        this.detector = new ImprovedSkystoneDetector(alliance);
        this.detector.useDefaults();
        camera.setPipeline(detector);
        camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
    }

    public SkystoneState getSkystoneState() {
        return detector.getSkystoneState();
    }

    public void stopPhoneCamDetector() {
        camera.stopStreaming();
    }

    public void stop() {
        requestOpModeStop();
    }

    public SkystoneHardware getRobot() {
        return this.getRobot(DEFAULT_START_POSE);
    }
}
