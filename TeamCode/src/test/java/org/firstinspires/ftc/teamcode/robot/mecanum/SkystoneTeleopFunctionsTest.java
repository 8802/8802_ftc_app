package org.firstinspires.ftc.teamcode.robot.mecanum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.simulator.SimulatedOpModeFactory;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.teleop.SkystoneRobotCentric;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SkystoneTeleopFunctionsTest {

    private void verifyMockMotorPower(DcMotorEx m, double power) {
        Mockito.verify(m).setPower(power);
    }

    @Test
    void opModeTest() {
        SimulatedOpModeFactory simOpMode = new SimulatedOpModeFactory(SkystoneRobotCentric.class);
        simOpMode.opMode.start();
        SkystoneHardware robot = simOpMode.robot;

        // At start, our position should be 0, 0 after waiting a bit
        simOpMode.elapseCycles(10, 100);
        assertEquals(simOpMode.robot.pose(), new Pose(0, 0, 0));
        assertEquals(simOpMode.robot.wheelPowers, new MecanumPowers(0,0, 0, 0));

        // Intake properly toggles
        simOpMode.cycle();
        //Mockito.verifyZeroInteractions(robot.intakeLeft, robot.intakeRight);
    }
}