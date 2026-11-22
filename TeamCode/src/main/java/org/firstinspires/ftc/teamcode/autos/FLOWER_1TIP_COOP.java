package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.commands.FollowPathCommand.fromCurrentPose;
import static org.firstinspires.ftc.teamcode.commands.FollowPathCommand.fromPose;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.EMPTY_MAGAZINE_TIME;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.FLOWER_RELOAD_TIME;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_INTAKE_FLOWER_1_FROM_PRELOAD_1;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_INTAKE_FLOWER_1_FROM_PRELOAD_2;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_PARK_FROM_FLOWER;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_START_FLOWER;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.SHOOTER_READY_TIME;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.SHOOTER_START_TIME;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class FLOWER_1TIP_COOP extends CompetitionAuto {
    @Override
    protected Pose startingPose() {
        return RED_START_FLOWER;
    }

    @Override
    protected Command buildRoute() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> distanceCorrection = 2.0),

                new WaitCommand(SHOOTER_START_TIME),
                new WaitCommand(5000),
                new InstantCommand(() -> intake.intakeFeed()),
                new WaitCommand(EMPTY_MAGAZINE_TIME),
                new InstantCommand(() -> intake.intakeStop()),

                new ParallelDeadlineGroup(
                        new WaitCommand(2500),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.shovelDown()),
                                fromPose(robot.motion, RED_START_FLOWER, RED_INTAKE_FLOWER_1_FROM_PRELOAD_1),
                                new InstantCommand(() -> intake.intakeIn()),
                                fromCurrentPose(robot.motion, RED_INTAKE_FLOWER_1_FROM_PRELOAD_2)
                        )
                ),
                new WaitCommand(FLOWER_RELOAD_TIME),
                new InstantCommand(() -> intake.intakeStop()),
                new WaitCommand(SHOOTER_READY_TIME),
                new InstantCommand(() -> intake.intakeFeed()),
                new WaitCommand(EMPTY_MAGAZINE_TIME),
                new InstantCommand(() -> intake.intakeStop()),
                new InstantCommand(() -> doneShooting = true),

                fromCurrentPose(robot.motion, RED_PARK_FROM_FLOWER),
                new WaitCommand(1000),

                new WaitCommand(100),
                new InstantCommand(robot.motion::cancel)
        );
    }
}
