package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;

abstract class Subroutine {
    abstract void run(MecanumHardware robot);
}

class ActivateIntake extends Subroutine {
    @Override
    void run(MecanumHardware robot) {
        robot.setIntakePower(1);
    }
}
