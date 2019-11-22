package org.firstinspires.ftc.teamcode.robot.mecanum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.simulator.SimulatedOpModeFactory;
import org.firstinspires.ftc.simulator.utils.MockDcMotorEx;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.DepositFlipper;
import org.firstinspires.ftc.teamcode.robot.mecanum.teleop.SkystoneRobotCentric;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SkystoneTeleopFunctionsTest {

    private void verifyMockMotorPower(DcMotorEx m, double power) {
        Mockito.verify(m).setPower(power);
    }

    @Test
    void opModeTest() {
        SimulatedOpModeFactory simOpMode = new SimulatedOpModeFactory(SkystoneRobotCentric.class);
        simOpMode.opMode.start();

        testStartingPosition(simOpMode, simOpMode.robot);
        testToggleServos(simOpMode, simOpMode.robot);

        //Mockito.verifyZeroInteractions(robot.intakeLeft, robot.intakeRight);
    }

    private void testStartingPosition(SimulatedOpModeFactory simOpMode, SkystoneHardware robot) {
        // At start, our position should be 0, 0 after waiting a bit
        simOpMode.elapseCycles(10, 100);
        assertEquals(simOpMode.robot.pose(), new Pose(0, 0, 0));
        assertEquals(simOpMode.robot.wheelPowers, new MecanumPowers(0,0, 0, 0));
        assertEquals(simOpMode.robot.wheelPowers, new MecanumPowers(0,0, 0, 0));

        // All our mechanisms should be in their default states
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, robot.lift.getMode());
        assertEquals(0, robot.lift.getTargetPosition());
        assertTrue(robot.lift.getPower() > 0);

        assertMotorOff(robot.intakeLeft);
        assertMotorOff(robot.intakeRight);

        // Ensure all servos are going to starting place
        assertEquals(DepositFlipper.LEFT_INTAKING, robot.blockFlipper.leftFlipper.getPosition());
        assertEquals(DepositFlipper.RIGHT_INTAKING, robot.blockFlipper.rightFlipper.getPosition());
        assertNotEquals(robot.blockFlipper.leftFlipper.getDirection(), robot.blockFlipper.rightFlipper.getDirection());
        assertEquals(SkystoneHardware.FOUNDATION_LATCH_OPEN, robot.leftFoundationLatch.servo.getPosition(), 0.1);
        assertEquals(SkystoneHardware.FOUNDATION_LATCH_OPEN, robot.rightFoundationLatch.servo.getPosition(), 0.1);
        assertNotEquals(robot.leftFoundationLatch.servo.getDirection(), robot.rightFoundationLatch.servo.getDirection());
        assertEquals(SkystoneHardware.CAPSTONE_RETRACTED, robot.capstoneDropper.servo.getPosition());
        assertEquals(SkystoneHardware.BLOCK_GRABBER_OPEN, robot.blockGrabber.servo.getPosition());
    }

    private void testToggleServos(SimulatedOpModeFactory simOpMode, SkystoneHardware robot) {
        /* Test capstone dropping toggle */
        assertEquals(SkystoneHardware.CAPSTONE_RETRACTED, robot.capstoneDropper.servo.getPosition());
        simOpMode.cycle();
        simOpMode.opMode.gamepad1.a = true;
        simOpMode.cycle();
        simOpMode.opMode.gamepad1.a = false;
        simOpMode.cycle();
        assertEquals(SkystoneHardware.CAPSTONE_DROPPED, robot.capstoneDropper.servo.getPosition());
        simOpMode.opMode.gamepad1.a = true;
        simOpMode.cycle();
        simOpMode.opMode.gamepad1.a = false;
        simOpMode.cycle();
        assertEquals(SkystoneHardware.CAPSTONE_RETRACTED, robot.capstoneDropper.servo.getPosition());

        /* Test foundation latch toggle */
        assertEquals(SkystoneHardware.FOUNDATION_LATCH_OPEN, robot.leftFoundationLatch.servo.getPosition());
        assertEquals(SkystoneHardware.FOUNDATION_LATCH_OPEN, robot.rightFoundationLatch.servo.getPosition());
        simOpMode.cycle();
        simOpMode.opMode.gamepad1.y = true;
        simOpMode.cycle();
        simOpMode.opMode.gamepad1.y = false;
        simOpMode.cycle();
        assertEquals(SkystoneHardware.FOUNDATION_LATCH_CLOSED, robot.leftFoundationLatch.servo.getPosition());
        assertEquals(SkystoneHardware.FOUNDATION_LATCH_CLOSED, robot.rightFoundationLatch.servo.getPosition());
        simOpMode.opMode.gamepad1.y = true;
        simOpMode.cycle();
        simOpMode.opMode.gamepad1.y = false;
        simOpMode.cycle();
        assertEquals(SkystoneHardware.FOUNDATION_LATCH_OPEN, robot.leftFoundationLatch.servo.getPosition());
        assertEquals(SkystoneHardware.FOUNDATION_LATCH_OPEN, robot.rightFoundationLatch.servo.getPosition());
    }

    private void assertMotorOff(DcMotorEx m) {
        assertEquals(0, m.getPower());
    }

}