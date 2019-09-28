package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.StopWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.firstinspires.ftc.teamcode.common.elements.SkystoneState;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.openftc.revextensions2.RevBulkData;

// Import constants
import static org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware.*;

public class SSAutoV1 extends SimulatableMecanumOpMode {

    MecanumHardware robot;
    PurePursuitPath followPath;

    public static SkystoneState SKYSTONE = SkystoneState.MIDDLE;

    // We'll place the robot right in the crook of the play field
    public Pose getStartingPosition() {
        return new Pose(-FIELD_RADIUS + 22.75, FIELD_RADIUS - 9, 3 * Math.PI / 2);
    }

    @Override
    public void init() {
        this.robot = this.getRobot();
        robot.initBNO055IMU(hardwareMap);
        followPath = new PurePursuitPath(robot,
                new Waypoint(getStartingPosition(), 4),

                // Attack the top left corner of the block
                new StopWaypoint(SKYSTONE.lowerX() + 4, SKYSTONE.y() + 2, 4, -0.75 * Math.PI, 2),

                // Drive up to and stop at foundation
                new Waypoint(SKYSTONE.lowerX() + 4, 48, 16),
                new Waypoint(48, 48, 16),
                new StopWaypoint(48, 24, 4, Math.PI/2, 2),

                // Drive back and grab the second skystone
                new Waypoint(48, 48, 16),
                new Waypoint(SKYSTONE.upperX() + 4, 48, 16),
                new StopWaypoint(SKYSTONE.upperX() + 4, SKYSTONE.y() + 2, 4,-0.75 * Math.PI, 2),

                // Drive up to and stop at foundation
                new Waypoint(SKYSTONE.upperX() + 4, 48, 16),
                new Waypoint(48, 48, 16),
                new StopWaypoint(48, 24, 4, Math.PI/2, 2),

                // Drive back and grab a third skystone
                new Waypoint(48, 48, 16),
                new Waypoint(-40, 48, 16),
                new StopWaypoint(-40, 24, 16, -0.75 * Math.PI, 2),
                new Waypoint(-40, 48, 16),
                new Waypoint(48, 48, 16),
                new StopWaypoint(48, 24, 4, Math.PI/2, 2),

                // And the fourth
                new Waypoint(48, 48, 16),
                new Waypoint(-48, 48, 16),
                new StopWaypoint(-48, 24, 16, -0.75 * Math.PI, 2),
                new Waypoint(-48, 48, 16),
                new Waypoint(48, 48, 16),
                new StopWaypoint(48, 24, 4, Math.PI/2, 2),

                // Drive up and place foundation in corner
                new Waypoint(48, 48, 16),
                new Waypoint(24, 48, 16),
                new StopWaypoint(FIELD_RADIUS - 27.5, FIELD_RADIUS - 17, 4, Math.PI, 2),

                // Park on the line
                new HeadingControlledWaypoint(FIELD_RADIUS - 27.5, 36, 16, Math.PI),
                new StopWaypoint(0, 36, 16, Math.PI, 2)
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
