package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.autoTurretAim;

/** Prepares a fixed shot while the route moves and feeds only when both commands are valid. */
public final class FixedShot {
    private final Shooter shooter;
    private final autoTurretAim turret;
    private final Intake intake;

    public FixedShot(Shooter shooter, autoTurretAim turret, Intake intake) {
        this.shooter = shooter;
        this.turret = turret;
        this.intake = intake;
    }

    public InstantCommand prepare(double velocity, double angleDeg) {
        return new InstantCommand(() -> {
            shooter.setMotorVelocity(velocity);
            turret.setAngleDegrees(angleDeg);
        });
    }

    public InstantCommand feed() {
        return new InstantCommand(() -> {
            // The route allows time for the servos to move. This check validates the commands.
            if (shooter.hasMotorVelocityTarget() && turret.isRequestedAngleInRange()) {
                intake.intakeFeed();
            } else {
                shooter.stop();
                intake.intakeStop();
            }
        });
    }
}
