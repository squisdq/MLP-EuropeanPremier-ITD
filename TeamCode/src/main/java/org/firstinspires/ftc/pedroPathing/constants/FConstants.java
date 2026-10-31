package org.firstinspires.ftc.pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftRear";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightRear";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 13.0;

        FollowerConstants.xMovement = (83.2 + 82.9 + 82.3 + 83.1 + 82.5) / 5;
        FollowerConstants.yMovement = (66.6 + 65.5 + 65.9 + 66.0 + 65.6) / 5;

        FollowerConstants.forwardZeroPowerAcceleration = -(38.1 + 34.4 + 35.6 + 36.7 + 38.5) / 5;
        FollowerConstants.lateralZeroPowerAcceleration = -(79.0 + 75.8 + 77.1 + 81.2 + 77.0) / 5;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.125, 0, 0.008, 0);
        FollowerConstants.translationalPIDFFeedForward = 0.014;
        FollowerConstants.translationalPIDFSwitch = 2;
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.155, 0, 0.012, 0);
        FollowerConstants.secondaryTranslationalPIDFFeedForward = 0.016;

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.75, 0, 0.08, 0);
        FollowerConstants.headingPIDFFeedForward = 0.008;
        FollowerConstants.useSecondaryHeadingPID = false;

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.02, 0, 0, 0.0002, 0);
        FollowerConstants.drivePIDFFeedForward = 0.012;
        FollowerConstants.drivePIDFSwitch = 5;
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.01, 0, 0, 0.0001, 0);
        FollowerConstants.secondaryDrivePIDFFeedForward = 0.008;

        FollowerConstants.zeroPowerAccelerationMultiplier = 6;
        FollowerConstants.centripetalScaling = 0.0004;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        FollowerConstants.useVelocityControlModeInTeleOp = true;
        FollowerConstants.useVoltageCompensationInTeleOp = true;
        FollowerConstants.useVoltageCompensationInAuto = true;
        FollowerConstants.useBrakeModeInTeleOp = true;
    }
}
