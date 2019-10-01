package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.StopWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.firstinspires.ftc.teamcode.common.elements.SkystoneState;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.openftc.revextensions2.RevBulkData;

import static org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware.FIELD_RADIUS;

// Import constants

public class SSAutoV2 extends SimulatableMecanumOpMode {

    Waypoint DEPOSIT_LOCATION = new StopWaypoint(9, 42, 4, Math.PI, 2);
    double GRAB_HEADING = -0.85 * Math.PI;

    MecanumHardware robot;
    PurePursuitPath followPath;

    public static SkystoneState SKYSTONE = SkystoneState.MIDDLE;

    // We'll place the robot right in the crook of the play field
    public Pose getStartingPosition() {
        return new Pose(-FIELD_RADIUS + 22.75 + 9, FIELD_RADIUS - 9, 3 * Math.PI / 2);
    }

    @Override
    public void init() {
        this.robot = this.getRobot();
        robot.initBNO055IMU(hardwareMap);
        followPath = new PurePursuitPath(robot,
                new Waypoint(getStartingPosition(), 4),

                // Grab the farthest block and move foundation into position
                new StopWaypoint(-FIELD_RADIUS + 4 + 4, 24, 4, -0.75 * Math.PI, 2),
                new Waypoint(-FIELD_RADIUS + 4 + 4, 48, 16),
                new Waypoint(0, 48, 16),
                new StopWaypoint(48, 32, 4, Math.PI/2, 2),
                new StopWaypoint(-3, 48, 8, Math.PI, 1),


                new StopWaypoint(-FIELD_RADIUS + 28 + 4, 24, 4, -0.75 * Math.PI, 2),
                new Waypoint(-FIELD_RADIUS + 28 + 4, 48, 16),
                DEPOSIT_LOCATION,

                new StopWaypoint(-FIELD_RADIUS + 42 + 4, 24, 4, GRAB_HEADING, 2),
                DEPOSIT_LOCATION,

                new StopWaypoint(-FIELD_RADIUS + 36 + 4, 24, 4, GRAB_HEADING, 2),
                DEPOSIT_LOCATION,

                new StopWaypoint(-FIELD_RADIUS + 20 + 4, 24, 4, GRAB_HEADING, 2),
                DEPOSIT_LOCATION,

                new StopWaypoint(-FIELD_RADIUS + 12 + 4, 24, 4, GRAB_HEADING, 2),
                DEPOSIT_LOCATION,

                // Drive up and place foundation in corner
                new HeadingControlledWaypoint(30, 36, 16, Math.PI),
                new HeadingControlledWaypoint(30, FIELD_RADIUS - 14, 16, Math.PI),
                new StopWaypoint(FIELD_RADIUS - 24, FIELD_RADIUS - 14, 4, Math.PI, 2),

                // Park on the line, pushing our partner
                new StopWaypoint(14, FIELD_RADIUS - 14, 16, Math.PI, 2)
        );
    }

    @Override
    public void start() {
        robot.initBulkReadTelemetry();
    }

    @Override
    public void loop() {
        RevBulkData data = robot.performBulkRead();
        if (!followPath.finished()) {
            followPath.update();
        } else {
            robot.setPowers(MecanumUtil.STOP);
        }
    }
}
