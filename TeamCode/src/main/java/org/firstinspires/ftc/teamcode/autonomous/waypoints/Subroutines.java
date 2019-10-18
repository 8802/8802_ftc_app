package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;

public class Subroutines {
    public interface Subroutine {
        void run(MecanumHardware robot);
    }

    public static final Subroutine ENABLE_INTAKE = (robot) -> { robot.setIntakePower(1); };
    public static final Subroutine STOP_INTAKE = (robot) -> { robot.setIntakePower(0); };
    public static final Subroutine REVERSE_INTAKE = (robot) -> { robot.setIntakePower(-1); };
}