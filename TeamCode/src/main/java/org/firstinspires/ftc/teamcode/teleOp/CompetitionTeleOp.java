package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.kernel.input.PadAxis.LEFT_STICK_RIGHT;
import static org.firstinspires.ftc.teamcode.kernel.input.PadAxis.LEFT_STICK_UP;
import static org.firstinspires.ftc.teamcode.kernel.input.PadAxis.RIGHT_STICK_RIGHT;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.kernel.Alliance;
import org.firstinspires.ftc.teamcode.kernel.KernelOpMode;
import org.firstinspires.ftc.teamcode.kernel.PoseStore;
import org.firstinspires.ftc.teamcode.kernel.profiles.RobotProfile;
import org.firstinspires.ftc.teamcode.subsystems.CompetitionProfile;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.autoTurretAim;

abstract class CompetitionTeleOp extends KernelOpMode {
    protected Drivetrain drivetrain;
    protected Intake intake;
    protected Shooter shooter;
    protected autoTurretAim turret;

    @Override
    protected final RobotProfile profile() {
        return CompetitionProfile.teleOp();
    }

    @Override
    protected final Telemetry display() {
        return new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    @Override
    protected void onInit() {
        drivetrain = robot.require(Drivetrain.class);
        intake = robot.require(Intake.class);
        shooter = robot.require(Shooter.class);
        turret = robot.require(autoTurretAim.class);
    }

    protected final Pose2D startingPoseFromAuto() {
        Pose saved = PoseStore.peek();
        if (saved == null) {
            // Keep the standalone fallback in the original Pinpoint units.
            return new Pose2D(DistanceUnit.INCH, 71.75, 9.0, AngleUnit.RADIANS, 90.0);
        }
        return new Pose2D(DistanceUnit.INCH, saved.getX(), saved.getY(), AngleUnit.RADIANS, saved.getHeading());
    }

    protected final Alliance allianceFromAuto() {
        Alliance saved = PoseStore.peekAlliance();
        return saved == null ? Alliance.RED : saved;
    }

    protected final void driveRobotCentric() {
        drivetrain.drive(robot.gp1.axis(LEFT_STICK_UP), robot.gp1.axis(LEFT_STICK_RIGHT),
                robot.gp1.axis(RIGHT_STICK_RIGHT), 1.0);
    }

    protected final void driveFieldCentric(double forwardHeadingDeg) {
        drivetrain.driveFieldOriented(robot.gp1.axis(LEFT_STICK_UP), robot.gp1.axis(LEFT_STICK_RIGHT),
                robot.gp1.axis(RIGHT_STICK_RIGHT), forwardHeadingDeg);
    }

    protected final void updateIntake(double triggerThreshold) {
        if (robot.gp1.rightTrigger.value() > triggerThreshold) {
            intake.intakeIn();
        } else if (robot.gp1.leftTrigger.value() > triggerThreshold) {
            intake.intakeOut();
        } else if (robot.gp1.rightBumper.isDown()) {
            intake.intakeFeed();
        } else {
            intake.intakeStop();
        }
    }
}
