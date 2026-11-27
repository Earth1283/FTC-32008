package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.kernel.Alliance;
import org.firstinspires.ftc.teamcode.kernel.input.Key;
import org.firstinspires.ftc.teamcode.subsystems.TargetSelector;

/** Field-centric driving with motion-compensated aim. The touchpad toggles manual driving. */
@TeleOp
public class tele extends CompetitionTeleOp {
    private boolean shooterOn;
    private boolean shovelDown;
    private boolean manual;
    private double leftDistance;
    private double leftTurret;
    private double panelPosition = 0.50;

    @Override
    protected void onInit() {
        super.onInit();
        robot.alliance(allianceFromAuto());
        drivetrain.setPosition(startingPoseFromAuto());
    }

    @Override
    protected void onStart() {
        turret.setEnabled(true);
    }

    @Override
    protected void onLoop() {
        if (robot.gp1.takePress(Key.TOUCHPAD)) manual = !manual;

        Pose2D current = drivetrain.getPosition();
        boolean red = robot.alliance() == Alliance.RED;
        boolean leftTarget = red == (current.getY(DistanceUnit.INCH) > TargetSelector.MID);
        boolean poseReady = drivetrain.getLocalizationStatus() == DeviceStatus.READY
                && Double.isFinite(current.getX(DistanceUnit.INCH))
                && Double.isFinite(current.getY(DistanceUnit.INCH))
                && Double.isFinite(current.getHeading(AngleUnit.DEGREES));
        double distanceCorrection = leftTarget ? leftDistance : 0.0;
        double turretCorrection = leftTarget ? leftTurret : 0.0;

        turret.setEnabled(manual || poseReady);
        if (manual) {
            driveRobotCentric();
            turret.setAngleDegrees(0.0);
        } else if (poseReady) {
            driveFieldCentric(red ? 0.0 : 180.0);
            Pose2D target = TargetSelector.getTargetForSide(red, leftTarget);
            turret.aimAtWithMotionCompensation(current, target, turretCorrection,
                    drivetrain.getVelocityXInchesPerSecond(), drivetrain.getVelocityYInchesPerSecond());
        } else {
            drivetrain.stop();
        }

        updateIntake(0);
        if (robot.gp1.takePress(Key.LEFT_BUMPER)) shooterOn = !shooterOn;
        if (shooterOn && (manual || poseReady)) {
            double distance = manual ? 40 : turret.getDistanceInches();
            shooter.setPollenVelocityByDistance(distance + distanceCorrection);
        } else {
            shooter.stop();
        }

        if (robot.gp1.takePress(Key.DPAD_UP) || robot.gp2.takePress(Key.DPAD_UP)) leftDistance += 2;
        if (robot.gp1.takePress(Key.DPAD_DOWN) || robot.gp2.takePress(Key.DPAD_DOWN)) leftDistance -= 2;
        if (robot.gp1.takePress(Key.DPAD_LEFT) || robot.gp2.takePress(Key.DPAD_LEFT)) leftTurret -= 2;
        if (robot.gp1.takePress(Key.DPAD_RIGHT) || robot.gp2.takePress(Key.DPAD_RIGHT)) leftTurret += 2;

        if (robot.gp1.takePress(Key.Y)) panelPosition += 0.01;
        if (robot.gp1.takePress(Key.A)) panelPosition -= 0.01;
        if (robot.gp2.takePress(Key.LEFT_BUMPER)) shovelDown = !shovelDown;
        if (shovelDown) intake.shovelDown();
        else intake.shovelUp();

        double appliedPosition = shooter.setPanel(panelPosition);
        if (Double.isFinite(appliedPosition)) panelPosition = appliedPosition;

        if (robot.telemetry.isDue()) {
            robot.telemetry.data("X", current.getX(DistanceUnit.INCH));
            robot.telemetry.data("Y", current.getY(DistanceUnit.INCH));
            robot.telemetry.data("H", current.getHeading(AngleUnit.DEGREES));
            robot.telemetry.data("pollenActualVelocity", shooter.getPollenVelocity());
            robot.telemetry.data("leftDistance", leftDistance);
            robot.telemetry.data("leftTurret", leftTurret);
            robot.telemetry.data("panelPos", appliedPosition);
        }
    }
}
