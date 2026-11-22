package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.commands.FollowPathCommand.fromCurrentPose;
import static org.firstinspires.ftc.teamcode.commands.FollowPathCommand.fromPose;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.EMPTY_MAGAZINE_TIME;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.FLOWER_RELOAD_TIME;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_INTAKE_FLOWER_1;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_INTAKE_FLOWER_2;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_INTAKE_FLOWER_2_CONTROL_FROM_GARDEN;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_INTAKE_GARDEN;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_INTAKE_GARDEN_2;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_INTAKE_TEAMMATE;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_INTAKE_TEAMMATE_2;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_INTAKE_TEAMMATE_CONTROL_1;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_PARK;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_PARK_CONTROL;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_PARK_FINAL;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_SHOOT_FLOWER_2;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_SHOOT_GARDEN;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_SHOOT_GARDEN_CONTROL_1;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_SHOOT_GARDEN_CONTROL_2;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_SHOOT_TEAMMATE;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_SHOOT_TEAMMATE_CONTROL_1;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_SHOOT_TEAMMATE_CONTROL_2;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_START;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.SHOOTER_READY_TIME;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.SHOOTER_START_TIME;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class GARDEN_3TIP_SOLO extends CompetitionAuto {
    @Override
    protected Pose startingPose() {
        return RED_START;
    }

    @Override
    protected Command buildRoute() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> distanceCorrection = 0.0),
                new InstantCommand(() -> degreeCorrection = 0.0),

                new WaitCommand(SHOOTER_START_TIME),
                new InstantCommand(() -> intake.intakeFeed()),
                new WaitCommand(EMPTY_MAGAZINE_TIME),
                new InstantCommand(() -> intake.intakeStop()),

                new ParallelDeadlineGroup(
                        new WaitCommand(2100),
                        new SequentialCommandGroup(
                                fromPose(robot.motion, RED_START, RED_INTAKE_GARDEN),
                                fromCurrentPose(robot.motion, RED_INTAKE_GARDEN_2).setMaxPower(0.4)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1800),
                                new InstantCommand(() -> intake.intakeIn())
                        )
                ),

                new InstantCommand(() -> degreeCorrection = -2.0),
                new ParallelCommandGroup(
                        fromCurrentPose(robot.motion, RED_SHOOT_GARDEN_CONTROL_1, RED_SHOOT_GARDEN_CONTROL_2, RED_SHOOT_GARDEN),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new InstantCommand(() -> intake.intakeStop()),
                                new WaitCommand(1500),
                                new InstantCommand(() -> intake.shovelDown())
                        )
                ),
                new WaitCommand(SHOOTER_READY_TIME),
                new InstantCommand(() -> intake.intakeFeed()),
                new WaitCommand(EMPTY_MAGAZINE_TIME),
                new InstantCommand(() -> intake.intakeIn()),

                new ParallelDeadlineGroup(
                        new WaitCommand(1000),
                        fromCurrentPose(robot.motion, RED_INTAKE_FLOWER_1)
                ),
                new WaitCommand(FLOWER_RELOAD_TIME),
                new InstantCommand(() -> intake.intakeStop()),

                new WaitCommand(SHOOTER_READY_TIME),
                new InstantCommand(() -> intake.intakeFeed()),
                new WaitCommand(EMPTY_MAGAZINE_TIME),
                new InstantCommand(() -> intake.intakeStop()),

                new InstantCommand(() -> degreeCorrection = 0.0),
                new ParallelDeadlineGroup(
                        new WaitCommand(2700),
                        new SequentialCommandGroup(
                                fromCurrentPose(robot.motion, RED_INTAKE_TEAMMATE_CONTROL_1, RED_INTAKE_TEAMMATE),
                                fromCurrentPose(robot.motion, RED_INTAKE_TEAMMATE_2)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> intake.shovelUp()),
                                new WaitCommand(500),
                                new InstantCommand(() -> intake.intakeIn())
                        )
                ),

                fromCurrentPose(robot.motion, RED_SHOOT_TEAMMATE_CONTROL_1, RED_SHOOT_TEAMMATE_CONTROL_2, RED_SHOOT_TEAMMATE),
                new WaitCommand(SHOOTER_READY_TIME),
                new InstantCommand(() -> intake.intakeFeed()),
                new WaitCommand(EMPTY_MAGAZINE_TIME),
                new InstantCommand(() -> intake.intakeStop()),
                new InstantCommand(() -> intake.shovelDown()),

                new ParallelDeadlineGroup(
                        new WaitCommand(2200),
                        fromCurrentPose(robot.motion, RED_INTAKE_FLOWER_2_CONTROL_FROM_GARDEN, RED_INTAKE_FLOWER_2),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new InstantCommand(() -> intake.intakeIn())
                        )
                ),
                new WaitCommand(FLOWER_RELOAD_TIME),
                new InstantCommand(() -> intake.intakeStop()),

                new ParallelCommandGroup(
                        fromCurrentPose(robot.motion, RED_SHOOT_FLOWER_2),
                        new SequentialCommandGroup(
                                new WaitCommand(800),
                                new InstantCommand(() -> intake.shovelUp())
                        )
                ),
                new InstantCommand(() -> intake.intakeFeed()),
                new WaitCommand(EMPTY_MAGAZINE_TIME),
                new InstantCommand(() -> intake.intakeStop()),
                new InstantCommand(() -> doneShooting = true),
                new InstantCommand(() -> shooter.stop()),

                fromCurrentPose(robot.motion, RED_PARK_CONTROL, RED_PARK),
                fromCurrentPose(robot.motion, RED_PARK_FINAL).setMaxPower(0.5),
                new WaitCommand(1000),

                new WaitCommand(100),
                new InstantCommand(robot.motion::cancel)
        );
    }
}
