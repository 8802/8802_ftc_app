package org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms;

import com.qualcomm.hardware.motors.NeveRest3_7GearmotorV1;
import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.ModernRoboticsMotorControllerParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFPositionParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * {@link NeveRest3_7GearmotorV1} represents the original v1 3-magnetic-pole version of the
 * NeveRest 3.7-geared gearmotor.
 */

// The system is built with the assumption that the motor and encoder turn the same way
// Since this isn't true in our setup, we need to wire both motors backward, and connect our single
// encoder to both motor encoder ports (we OC don't need to connect 5V and GND twice, though)

// TicksPerRev is actually 8192, but since the rev hub stores values in a 16-bit short, actually
// providing the true value would lead to an overflow. Instead, we reduce it by a factor of 10,
// and compensate in our PID parameters
@MotorType(ticksPerRev=819.2, gearing=5, maxRPM=2800, orientation= Rotation.CCW)
@DeviceProperties(xmlTag="ExternalLiftMotor", name="External 5:1 Lift Motor", builtIn = true)
@DistributorInfo(distributor="AndyMark", model="am-3461")
@ExpansionHubPIDFVelocityParams(P=0.15, I=0.017, F=1.8)
@ExpansionHubPIDFPositionParams(P=5.0)
public interface ExternalLiftMotor {

}
