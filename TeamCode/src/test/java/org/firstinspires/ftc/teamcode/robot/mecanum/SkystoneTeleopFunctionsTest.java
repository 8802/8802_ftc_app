package org.firstinspires.ftc.teamcode.robot.mecanum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.simulator.SimulatedOpModeFactory;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.DepositFlipper;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.SimpleLift;
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
        SimulatedOpModeFactory simOpMode = new SimulatedOpModeFactory(SkystoneTeleop.class);
        simOpMode.opMode.start();

        testStartingPosition(simOpMode, simOpMode.robot);
        testToggleServos(simOpMode, simOpMode.robot);
        simulateCycle(simOpMode, simOpMode.robot);
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

    private void simulateCycle(SimulatedOpModeFactory simOpMode, SkystoneHardware robot) {
        // Verify we're intaking right now
        assertMotorOff(robot.intakeLeft);
        assertMotorOff(robot.intakeRight);

        simOpMode.opMode.gamepad1.left_stick_button = true;
        simOpMode.cycle();
        simOpMode.opMode.gamepad1.left_stick_button = false;
        simOpMode.cycle();

        assertEquals(1, robot.intakeLeft.getPower());
        assertEquals(1, robot.intakeRight.getPower());
        simOpMode.elapseCycles(2, 100);

        // Now simulate block pickup
        simOpMode.robot.dataGen.analogInputs[SkystoneHardware.TRAY_DETECTOR_PORT] = 10000;
        simOpMode.cycle();

        // Intake should automatically turn off
        assertMotorOff(robot.intakeLeft);
        assertMotorOff(robot.intakeRight);
        assertEquals(DepositFlipper.LEFT_INTAKING, robot.blockFlipper.leftFlipper.getPosition());
        assertEquals(DepositFlipper.RIGHT_INTAKING, robot.blockFlipper.rightFlipper.getPosition());

        // Now we grab the block
        simOpMode.opMode.gamepad1.right_trigger = 1;
        simOpMode.cycle();
        simOpMode.opMode.gamepad1.right_trigger = 0;
        simOpMode.cycle();

        // We should be trying to grab the block
        assertEquals(DepositFlipper.LEFT_GRABBING, robot.blockFlipper.leftFlipper.getPosition());
        assertEquals(DepositFlipper.RIGHT_GRABBING, robot.blockFlipper.rightFlipper.getPosition());

        simOpMode.elapseCycles(2, 100);
        assertEquals(DepositFlipper.LEFT_DRIVING, robot.blockFlipper.leftFlipper.getPosition());
        assertEquals(DepositFlipper.RIGHT_DRIVING, robot.blockFlipper.rightFlipper.getPosition());
        assertEquals(SkystoneHardware.BLOCK_GRABBER_CLOSED, robot.blockGrabber.servo.getPosition());

        simOpMode.opMode.gamepad1.right_trigger = 1;
        simOpMode.cycle();
        simOpMode.opMode.gamepad1.right_trigger = 0;
        simOpMode.cycle();

        assertEquals(DepositFlipper.LEFT_NORM_EXTEND, robot.blockFlipper.leftFlipper.getPosition());
        assertEquals(DepositFlipper.RIGHT_NORM_EXTEND, robot.blockFlipper.rightFlipper.getPosition());
        assertEquals(SkystoneHardware.BLOCK_GRABBER_CLOSED, robot.blockGrabber.servo.getPosition());
        assertEquals(0, robot.lift.getTargetPosition());

        // Move lift up two levels
        simOpMode.opMode.gamepad1.dpad_right = true;
        simOpMode.cycle();
        simOpMode.opMode.gamepad1.dpad_right = false;
        simOpMode.cycle();
        simOpMode.opMode.gamepad1.dpad_right = true;
        simOpMode.cycle();
        simOpMode.opMode.gamepad1.dpad_right = false;
        simOpMode.cycle();
        assertEquals(SimpleLift.LAYER_0 + SimpleLift.LAYER_SHIFT * 2, robot.lift.getTargetPosition());

        /* Place block */
        simOpMode.opMode.gamepad1.right_trigger = 1;
        simOpMode.cycle();
        simOpMode.opMode.gamepad1.right_trigger = 0;
        simOpMode.cycle();

        // Ensure lift is moving up, but we're not retracting the servos yet
        assertTrue(SimpleLift.LAYER_0 + SimpleLift.LAYER_SHIFT * 2 < robot.lift.getTargetPosition());
        assertEquals(SkystoneHardware.BLOCK_GRABBER_OPEN, robot.blockGrabber.servo.getPosition());
        assertEquals(DepositFlipper.LEFT_NORM_EXTEND, robot.blockFlipper.leftFlipper.getPosition());
        assertEquals(DepositFlipper.RIGHT_NORM_EXTEND, robot.blockFlipper.rightFlipper.getPosition());

        // Now once sequence is over, we should be properly reset
        simOpMode.elapseCycles(2, 100);
        assertEquals(SimpleLift.LAYER_0, robot.lift.getTargetPosition());
        assertEquals(SkystoneHardware.BLOCK_GRABBER_OPEN, robot.blockGrabber.servo.getPosition());
        assertEquals(DepositFlipper.LEFT_INTAKING, robot.blockFlipper.leftFlipper.getPosition());
        assertEquals(DepositFlipper.RIGHT_INTAKING, robot.blockFlipper.rightFlipper.getPosition());
        assertEquals(1, robot.intakeLeft.getPower());
        assertEquals(1, robot.intakeRight.getPower());
    }

    private void assertMotorOff(DcMotorEx m) {
        assertEquals(0, m.getPower());
    }

}