package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;

@Config
public class Subroutines {
    public interface Subroutine {}

    public interface OnceOffSubroutine extends Subroutine {
        void runOnce(MecanumHardware robot);
    }

    public interface RepeatedSubroutine extends Subroutine {
        boolean runLoop(MecanumHardware robot); // Returns whether we should advance
    }

    // Once a position is reached, if that position has an ArrivalInterruptSubroutine, we advance to
    // the next waypoint (so the waypoint with the ArrivalInterruptSubroutine is our current waypoint)
    // and we call runCycle every tick until it eventually returns true
    public interface ArrivalInterruptSubroutine extends Subroutine {
        boolean runCycle(MecanumHardware robot); // Returns whether it's complete
    }

    public static final OnceOffSubroutine ENABLE_INTAKE = (robot) -> { robot.setIntakePower(1); };
    public static final OnceOffSubroutine STOP_INTAKE = (robot) -> { robot.setIntakePower(0); };
    public static final OnceOffSubroutine REVERSE_INTAKE = (robot) -> { robot.setIntakePower(-1); };

    public static double BLOCK_GRAB_POWER_THRESHOLD = 3000;
    public static final RepeatedSubroutine CHECK_BLOCK_GRAB = (robot) -> {
        if (robot.lastIntakeCurrent > BLOCK_GRAB_POWER_THRESHOLD) {
            //robot.setIntakePower(0); // Disable intake
            return true; // Advance to next motion path
        }
        return false;
    };
}