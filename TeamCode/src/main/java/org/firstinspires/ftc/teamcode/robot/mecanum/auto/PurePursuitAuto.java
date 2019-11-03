package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.firstinspires.ftc.teamcode.common.elements.Alliance;
import org.firstinspires.ftc.teamcode.common.elements.SkystoneState;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.auto.vision.ImprovedSkystoneDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.revextensions2.RevBulkData;

import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware.FIELD_RADIUS;

@Config
public abstract class PurePursuitAuto extends SimulatableMecanumOpMode {
    Pose DEFAULT_START_POSITION = new Pose(-FIELD_RADIUS + 22.75 + 9, FIELD_RADIUS - 9, 3 * Math.PI / 2);

    public static int CAMERA_WIDTH = 800;
    public static int CAMERA_HEIGHT = 448;


    SkystoneHardware robot;
    PurePursuitPath followPath;

    ImprovedSkystoneDetector detector;
    OpenCvCamera webcam;

    // Robot state
    public static SkystoneState SKYSTONE = SkystoneState.UPPER;
    public static Alliance ALLIANCE = Alliance.BLUE;

    public abstract Pose getBlueStartPosition();
    public abstract List<Waypoint> getPurePursuitWaypoints();

    @Override
    public void init() {
        Pose start = getBlueStartPosition();
        /*if (ALLIANCE == Alliance.RED) {
            start.y *= -1;
            start.heading *= MathUtil.angleWrap(-start.heading);
        }*/
        this.robot = this.getRobot(start);

        // Start camera
        /*robot.initCamera(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(robot.camera, cameraMonitorViewId);
        webcam.openCameraDevice();
        this.detector = new ImprovedSkystoneDetector();
        this.detector.useDefaults();
        webcam.setPipeline(detector);
        webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);*/

        followPath = new PurePursuitPath(robot, getPurePursuitWaypoints());
        // We design all paths for blue side, and then flip them for red
        if (ALLIANCE == Alliance.RED) {
            followPath.reverse();
        }
        telemetry.clearAll();
    }

    @Override
    public void init_loop() {
        /*telemetry.addData("Block location", detector.getSkystoneState().toString());
        telemetry.addData("Stone Position X", detector.getScreenPosition().x);
        telemetry.addData("Stone Position Y", detector.getScreenPosition().y);
        telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam.getFps()));
        telemetry.update();*/
    }

    @Override
    public void start() {
        telemetry.clearAll();
        robot.initBulkReadTelemetry();
        //SKYSTONE = detector.getSkystoneState();
        //webcam.stopStreaming();
    }

    @Override
    public void loop() {
        RevBulkData data = robot.performBulkRead();
        robot.drawDashboardPath(followPath);
        robot.sendDashboardTelemetryPacket();

        if (!followPath.finished()) {
            followPath.update();
        } else {
            robot.setPowers(MecanumUtil.STOP);
            stop();
        }
    }
}
