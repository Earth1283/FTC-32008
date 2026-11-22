package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.kernel.Alliance;
import org.firstinspires.ftc.teamcode.kernel.input.Key;
import org.firstinspires.ftc.teamcode.subsystems.TargetSelector;

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
        robot.alliance(Alliance.RED);
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
        boolean leftTarget = current.getY(DistanceUnit.INCH) > TargetSelector.MID;
        double distanceCorrection = leftTarget ? leftDistance : 0.0;
        double turretCorrection = leftTarget ? leftTurret : 0.0;

        if (manual) {
            driveRobotCentric();
            turret.setAngleDegrees(0.0);
        } else {
            driveFieldCentric(0.0);
            Pose2D target = TargetSelector.getTargetForSide(true, leftTarget);
            turret.aimAtWithMotionCompensation(current, target, turretCorrection,
                    drivetrain.getVelocityXInchesPerSecond(), drivetrain.getVelocityYInchesPerSecond());
        }

        updateIntake(0);
        if (robot.gp1.takePress(Key.LEFT_BUMPER)) shooterOn = !shooterOn;
        if (shooterOn) {
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
