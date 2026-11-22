package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14)
            .forwardZeroPowerAcceleration(-25.060886780276423)
            .lateralZeroPowerAcceleration(-48.2365845444232)
            .useSecondaryDrivePIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryTranslationalPIDF(false)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.018, 0.0, 0.0005, 0.6, 0.0))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.13, 0.0, 0.025, 0.0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.1, 0.0, 0.11, 0.0))
            .centripetalScaling(0.0005)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.08, 0.041779444580625644, 0.002244990632450207))
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.97, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftRearMotorName("lb")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(75.97025617464321)
            .yVelocity(59.77761432317298)
            .useVoltageCompensation(true)
            ;

    // Autonomous Pinpoint offsets are in millimeters. TeleOp keeps its own calibration.
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-138)
            .strafePodX(-33)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pp")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            ;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
