package org.firstinspires.ftc.simulator;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.VirtualMecanumHardware;
import org.mockito.Mockito;
import org.mockito.invocation.InvocationOnMock;
import org.mockito.stubbing.Answer;

public class SimulatedOpModeFactory {
    double FRAMERATE = 20;
    final static int INIT_ITERATIONS = 40;
    final static int RUN_ITERATIONS = 400;

    public SimulatableMecanumOpMode opMode; // Pointer to our op mode
    public VirtualMecanumHardware robot; // Pointer to our virtual robot
    public boolean stopRequested;

    public SimulatedOpModeFactory(Class c) {
        // Assert we were passed an opmode we can use
        assert SimulatableMecanumOpMode.class.isAssignableFrom(c);

        /*When we instantiate a MecanumHardware object, we can optionally tell it where its start
        position is (otherwise, it defaults to zero). This defaulting case occurs in
        SimulatableMecanumOpMode, so we only deal with the case where we're passed a position. We
        override the getRobot method to take the position, intantiate a virtual robot, save that to
        SimulatedOpModeFactory.robot, and return it as well.
         */
        opMode = (SimulatableMecanumOpMode) Mockito.spy(c);
        stopRequested = false;

        this.robot = new VirtualMecanumHardware(new Pose(0, 0, 0));
        Mockito.doAnswer((Answer<MecanumHardware>) invocation -> {
            Object[] args = invocation.getArguments();

            // Instantiate a virtual robot based on the given start position
            this.robot = new VirtualMecanumHardware((Pose) args[0]);
            return this.robot;
        }).when(opMode).getRobot(Mockito.any());

        Mockito.doAnswer(invocation -> {
            stopRequested = true;
            return null;
        }).when(opMode).stop();

        /*then((Answer) invocation -> {
            Object[] args = invocation.getArguments();

            // Instantiate a virtual robot based on the given start position
            this.robot = new VirtualMecanumHardware((Pose) args[0]);
            return this.robot;
        });*/
        /*Mockito.doAnswer(invocation -> {
            this.running = false;
            return null;
        }).when(opMode).requestOpModeStop();*/

        // Mock the gamepads
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();

        opMode.init();
        opMode.init_loop();
    }

    public void elapseCycles(double seconds, double loopHertz) {
        int iterations = (int) Math.round(seconds * loopHertz);
        for (int i = 0; i < iterations; i++) {
            opMode.loop();
            robot.elapse(seconds / iterations);
        }
    }
}
