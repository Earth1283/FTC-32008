package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.commands.FollowPathCommand.fromCurrentPose;
import static org.firstinspires.ftc.teamcode.commands.FollowPathCommand.fromPose;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.EMPTY_MAGAZINE_TIME;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_PARK_E;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_PARK_M;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_PARK_S;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.SHOOTER_START_TIME;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Flower - Preload and Park")
public class FLOWER_0TIP_COOP extends CompetitionAuto {
    @Override
    protected Pose startingPose() {
        return RED_PARK_S;
    }

    @Override
    protected Command buildRoute() {
        return new SequentialCommandGroup(
                // Spin up the flywheel and wait for the cooperative shooting window before feeding.
                // Shoot the preload. Flywheel speed is in ticks per second and turret angle is relative to the chassis.
                fixedShot.prepare(1140, -90), // Verify this angle on the robot.
                new WaitCommand(SHOOTER_START_TIME),
                new WaitCommand(3400),
                fixedShot.feed(),
                new WaitCommand(EMPTY_MAGAZINE_TIME + 200),
                new InstantCommand(intake::intakeStop),
                new InstantCommand(shooter::stop),

                fromPose(robot.motion, RED_PARK_S, RED_PARK_M),
                fromCurrentPose(robot.motion, RED_PARK_E),
                new InstantCommand(this::requestOpModeStop)
        );
    }
}
