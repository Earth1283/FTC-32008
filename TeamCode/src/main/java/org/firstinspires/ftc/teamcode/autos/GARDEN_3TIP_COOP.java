package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.commands.FollowPathCommand.fromCurrentPose;
import static org.firstinspires.ftc.teamcode.commands.FollowPathCommand.fromPose;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.EMPTY_MAGAZINE_TIME;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.FLOWER_RELOAD_TIME;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_INTAKE_FLOWER_2;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_INTAKE_FLOWER_2_CONTROL_FROM_GARDEN;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_INTAKE_GARDEN;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_INTAKE_GARDEN_2;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_PARK;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_PARK_CONTROL;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_PARK_FINAL;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_SHOOT_FLOWER_2;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_SHOOT_TEAMMATE;
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

@Autonomous(name = "Garden - 3 Tips Cooperative")
public class GARDEN_3TIP_COOP extends CompetitionAuto {
    @Override
    protected Pose startingPose() {
        return RED_START;
    }

    @Override
    protected Command buildRoute() {
        return new SequentialCommandGroup(
                // Spin up for the preload shot.
                fixedShot.prepare(1120, 89.6),
                new WaitCommand(SHOOTER_START_TIME),
                fixedShot.feed(),
                new WaitCommand(EMPTY_MAGAZINE_TIME),
                new InstantCommand(intake::intakeStop),

                // Slow down through Garden while the intake starts on a delay.
                new ParallelDeadlineGroup(
                        new WaitCommand(2500),
                        new SequentialCommandGroup(
                                fromPose(robot.motion, RED_START, RED_INTAKE_GARDEN),
                                fromCurrentPose(robot.motion, RED_INTAKE_GARDEN_2).setMaxPower(0.4)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1800),
                                new InstantCommand(intake::intakeIn)
                        )
                ),
                new WaitCommand(1000),
                new InstantCommand(intake::intakeStop),
                new WaitCommand(9000),

                // Drive to the shooting position with the Garden pieces, then pause briefly before feeding.

                // Shoot the Garden pieces from the partner side.
                fixedShot.prepare(1120, 89.5),
                fromCurrentPose(robot.motion, RED_SHOOT_TEAMMATE),
                new WaitCommand(SHOOTER_READY_TIME),
                fixedShot.feed(),
                new WaitCommand(EMPTY_MAGAZINE_TIME),
                new InstantCommand(intake::intakeStop),
                new InstantCommand(intake::shovelDown),

                // Follow the curve into Flower 2 while collecting within the timed window.
                new ParallelDeadlineGroup(
                        new WaitCommand(2800),
                        fromCurrentPose(robot.motion, RED_INTAKE_FLOWER_2_CONTROL_FROM_GARDEN, RED_INTAKE_FLOWER_2),
                        new SequentialCommandGroup(
                                new WaitCommand(2000),
                                new InstantCommand(intake::intakeIn)
                        )
                ),
                new WaitCommand(FLOWER_RELOAD_TIME),
                new InstantCommand(intake::intakeStop),

                // Raise the shovels while moving to the Flower 2 shooting position.
                fixedShot.prepare(1120, 89.5),
                new ParallelCommandGroup(
                        fromCurrentPose(robot.motion, RED_SHOOT_FLOWER_2),
                        new SequentialCommandGroup(
                                new WaitCommand(800),
                                new InstantCommand(intake::shovelUp)
                        )
                ),
                fixedShot.feed(),
                new WaitCommand(EMPTY_MAGAZINE_TIME),
                new InstantCommand(intake::intakeStop),
                new InstantCommand(shooter::stop),

                // Drive to the parking point and let the robot settle before handing off its pose.
                fromCurrentPose(robot.motion, RED_PARK_CONTROL, RED_PARK),
                fromCurrentPose(robot.motion, RED_PARK_FINAL).setMaxPower(0.5),
                new WaitCommand(1000),

                new WaitCommand(100),
                new InstantCommand(this::requestOpModeStop)
        );
    }
}
