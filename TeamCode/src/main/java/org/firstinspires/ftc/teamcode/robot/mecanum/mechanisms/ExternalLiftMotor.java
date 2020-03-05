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

//
@MotorType(ticksPerRev=145.6, gearing=5.2, maxRPM=900, orientation=Rotation.CCW)
@DeviceProperties(xmlTag="ExternalLiftMotor", name="GoBuilda 5.2:1 Lift Motor", builtIn = true)
@DistributorInfo(distributor="goBILDA_distributor", model="goBILDA-5202")
@ExpansionHubPIDFVelocityParams(P=1.17, I=0.117, F=11.7)
@ExpansionHubPIDFPositionParams(P=5.0)
public interface ExternalLiftMotor {

}
