package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Config
public class DepositUntilSuccessful implements Subroutines.RepeatedSubroutine {

    public static double NO_NEW_CYCLES_DEADLINE = 23; // Seconds

    static ElapsedTime timeSinceStart = new ElapsedTime();
    ElapsedTime attemptTime;
    int attempt;


    public DepositUntilSuccessful() {
        this.attemptTime = null;
        this.attempt = 0;
        timeSinceStart.reset();
    }

    @Override
    public boolean runLoop(SkystoneHardware robot, PurePursuitPath path) {
        if (attemptTime == null) {
            attemptTime = new ElapsedTime();
            robot.actionCache.add(new DelayedSubroutine(425, Subroutines.SET_FLIPPER_MAX_EXTEND));
            robot.actionCache.add(new DelayedSubroutine(300, (r) -> {r.pidLift.setLayer(2);}));
            robot.actionCache.add(new DelayedSubroutine(1050, Subroutines.OPEN_CLAW));
            robot.actionCache.add(new DelayedSubroutine(1050, (r) -> r.pidLift.lift.setPower(1)));
            robot.actionCache.add(new DelayedSubroutine(1200, Subroutines.SET_FLIPPER_INTAKING));
            robot.actionCache.add(new DelayedSubroutine(1650, Subroutines.LOWER_LIFT_TO_GRABBING));
            attempt = 1;
        }

        int msToDriveAway = (attempt == 1) ? 1400 : 3000;
        if (attemptTime.milliseconds() > msToDriveAway && !robot.hasBlockInTray()) {
            boolean skipped = optionallySkip(robot, path);
            // If we're not skipping the remaining paths, return true
            // If we are skipping remaining paths, return false as we'll leave anyway
            return !skipped;
        }

        if (attemptTime.milliseconds() > 2000 && robot.hasBlockInTray()) {
            boolean skipped = optionallySkip(robot, path);
            if (!skipped) {
                attemptTime.reset();
                replaceBlock(robot);
                attempt += 1;
            }
        }

        return false;
    }

    private void replaceBlock(SkystoneHardware robot) {
        robot.blockFlipper.readyBlockGrab();
        robot.blockGrabber.extend(); // Grab the block
        robot.pidLift.cacheToGrabbing();
        robot.actionCache.add(new DelayedSubroutine(600, Subroutines.SET_FLIPPER_NORM_EXTEND));
        robot.actionCache.add(new DelayedSubroutine(1400, Subroutines.OPEN_CLAW));
        robot.actionCache.add(new DelayedSubroutine(1650, Subroutines.LIFT_A_LITTLE));
        robot.actionCache.add(new DelayedSubroutine(2150, Subroutines.SET_FLIPPER_INTAKING));
        robot.actionCache.add(new DelayedSubroutine(2150, Subroutines.LOWER_LIFT_TO_GRABBING));
    }

    private boolean optionallySkip(SkystoneHardware robot, PurePursuitPath path) {
        if (timeSinceStart.seconds() > NO_NEW_CYCLES_DEADLINE) {
            robot.actionCache.clear();
            robot.actionCache.add(new DelayedSubroutine(400, Subroutines.LOWER_LIFT_TO_GRABBING));
            path.currPoint = path.waypoints.size() - 2;
            return true;
        }
        return false;
    }
}
