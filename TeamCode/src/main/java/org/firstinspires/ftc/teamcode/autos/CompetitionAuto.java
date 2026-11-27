package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.FixedShot;
import org.firstinspires.ftc.teamcode.kernel.Alliance;
import org.firstinspires.ftc.teamcode.kernel.KernelOpMode;
import org.firstinspires.ftc.teamcode.kernel.profiles.RobotProfile;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.CompetitionProfile;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.autoTurretAim;

abstract class CompetitionAuto extends KernelOpMode {
    private Follower follower;
    private CommandScheduler commands;
    private TelemetryManager panels;
    private autoTurretAim turret;
    protected Intake intake;
    protected Shooter shooter;
    protected FixedShot fixedShot;

    protected abstract Pose startingPose();

    protected abstract Command buildRoute();

    @Override
    protected final RobotProfile profile() {
        return CompetitionProfile.autonomous(this::createFollower);
    }

    private Follower createFollower(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose().copy());
        return follower;
    }

    @Override
    protected final void onInit() {
        robot.alliance(Alliance.RED);
        shooter = robot.require(Shooter.class);
        intake = robot.require(Intake.class);
        turret = robot.require(autoTurretAim.class);
        fixedShot = new FixedShot(shooter, turret, intake);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();
        intake.shovelUp();
        panels.update();
    }

    @Override
    protected final void onInitLoop() {
        Drawing.drawRobot(robot.motion.pose());
        Drawing.sendPacket();
        panels.update();
    }

    @Override
    protected final void onStart() {
        turret.setEnabled(true);
        follower.activateAllPIDFs();
        commands = CommandScheduler.getInstance();
        commands.schedule(buildRoute());
    }

    @Override
    protected final void onLoop() {
        publishPose();
        commands.run();
        Drawing.drawDebug(follower);
        panels.update();
    }

    private void publishPose() {
        Pose pose = robot.motion.pose();
        panels.addData("x", pose.getX());
        panels.addData("y", pose.getY());
        panels.addData("heading", Math.toDegrees(pose.getHeading()));
    }

    @Override
    protected final void onStop() {
        if (commands != null) {
            commands.cancelAll();
            commands.reset();
        }
    }
}
