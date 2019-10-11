package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

@Autonomous
public class SSAutoV1 extends SimulatableMecanumOpMode {

    Waypoint DEPOSIT_LOCATION = new StopWaypoint(24, 30, 4, 0.75 * Math.PI, 2);
    // We'll place the robot right in the crook of the play field
    Pose START_POSITION = new Pose(-FIELD_RADIUS + 22.75, FIELD_RADIUS - 9, 3 * Math.PI / 2);
    MecanumHardware robot;
    PurePursuitPath followPath;

    public static SkystoneState SKYSTONE = SkystoneState.MIDDLE;

    @Override
    public void init() {
        this.robot = this.getRobot(START_POSITION);
        robot.initBNO055IMU(hardwareMap);
        followPath = new PurePursuitPath(robot,
                new Waypoint(START_POSITION, 4),

                // Attack the top left corner of the block
                new StopWaypoint(SKYSTONE.lowerX() + 4, SKYSTONE.y() + 2, 4, -0.75 * Math.PI, 2),
                new Waypoint(0, 48, 16),
                DEPOSIT_LOCATION,

                // Drive back and grab the second skystone
                new Waypoint(0, 48, 16),
                new StopWaypoint(SKYSTONE.upperX() + 4, SKYSTONE.y() + 2, 4,-0.75 * Math.PI, 2),
                new Waypoint(0, 48, 16),
                DEPOSIT_LOCATION,

                // Drive back and grab a third skystone
                new Waypoint(0, 48, 16),
                new StopWaypoint(-40, 24, 16, -0.75 * Math.PI, 2),
                new Waypoint(0, 48, 16),
                DEPOSIT_LOCATION,

                // And the fourth
                new Waypoint(0, 48, 16),
                new StopWaypoint(-48, 24, 16, -0.75 * Math.PI, 2),
                new Waypoint(0, 48, 16),
                DEPOSIT_LOCATION,

                // Drive up and place foundation in corner
                new HeadingControlledWaypoint(36, 40, 4, Math.PI/2),
                new StopWaypoint(48, 32, 4, Math.PI/2, 2),
                new HeadingControlledWaypoint(36, 48, 16, Math.PI   ),
                new StopWaypoint(FIELD_RADIUS - 27.5, FIELD_RADIUS - 14, 4, Math.PI, 2),

                // Park on the line
                new HeadingControlledWaypoint((FIELD_RADIUS - 27.5) / 2, FIELD_RADIUS - 14, 16, Math.PI),
                new HeadingControlledWaypoint((FIELD_RADIUS - 27.5) / 2, 36, 16, Math.PI),
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
