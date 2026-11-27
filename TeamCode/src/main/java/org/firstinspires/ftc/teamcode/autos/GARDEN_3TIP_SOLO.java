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

@Autonomous(name = "Garden - 3 Tips Solo")
public class GARDEN_3TIP_SOLO extends CompetitionAuto {
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
                        new WaitCommand(2100),
                        new SequentialCommandGroup(
                                fromPose(robot.motion, RED_START, RED_INTAKE_GARDEN),
                                fromCurrentPose(robot.motion, RED_INTAKE_GARDEN_2).setMaxPower(0.4)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1800),
                                new InstantCommand(intake::intakeIn)
                        )
                ),

                // Prepare the Garden shot while driving to the shooting position.
                fixedShot.prepare(1120, 104.5),
                new ParallelCommandGroup(
                        fromCurrentPose(robot.motion, RED_SHOOT_GARDEN_CONTROL_1, RED_SHOOT_GARDEN_CONTROL_2, RED_SHOOT_GARDEN),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new InstantCommand(intake::intakeStop),
                                new WaitCommand(1500),
                                new InstantCommand(intake::shovelDown)
                        )
                ),
                new WaitCommand(SHOOTER_READY_TIME),
                fixedShot.feed(),
                new WaitCommand(EMPTY_MAGAZINE_TIME),
                new InstantCommand(intake::intakeIn),

                // Flower 1 needs about 164° at the current endpoint. Feeding is skipped until calibrated.
                fixedShot.prepare(1120, Double.NaN),
                // Drive to the Flower 1 collection area within the timed window.
                new ParallelDeadlineGroup(
                        new WaitCommand(1000),
                        fromCurrentPose(robot.motion, RED_INTAKE_FLOWER_1)
                ),
                new WaitCommand(FLOWER_RELOAD_TIME),
                new InstantCommand(intake::intakeStop),

                // Pause briefly after collecting, then open the gate and feed.
                new WaitCommand(SHOOTER_READY_TIME),
                fixedShot.feed(),
                new WaitCommand(EMPTY_MAGAZINE_TIME),
                new InstantCommand(intake::intakeStop),

                // Follow the curve to the partner-side collection area, raising the shovels and starting the intake along the way.
                new ParallelDeadlineGroup(
                        new WaitCommand(2700),
                        new SequentialCommandGroup(
                                fromCurrentPose(robot.motion, RED_INTAKE_TEAMMATE_CONTROL_1, RED_INTAKE_TEAMMATE),
                                fromCurrentPose(robot.motion, RED_INTAKE_TEAMMATE_2)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(intake::shovelUp),
                                new WaitCommand(500),
                                new InstantCommand(intake::intakeIn)
                        )
                ),

                // Drive to the partner-side shooting position, then feed for the specified time.
                // Shoot from the partner side.
                fixedShot.prepare(1120, 89.5),
                fromCurrentPose(robot.motion, RED_SHOOT_TEAMMATE_CONTROL_1, RED_SHOOT_TEAMMATE_CONTROL_2, RED_SHOOT_TEAMMATE),
                new WaitCommand(SHOOTER_READY_TIME),
                fixedShot.feed(),
                new WaitCommand(EMPTY_MAGAZINE_TIME),
                new InstantCommand(intake::intakeStop),
                new InstantCommand(intake::shovelDown),

                // Follow the curve into Flower 2 while collecting within the timed window.
                new ParallelDeadlineGroup(
                        new WaitCommand(2200),
                        fromCurrentPose(robot.motion, RED_INTAKE_FLOWER_2_CONTROL_FROM_GARDEN, RED_INTAKE_FLOWER_2),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
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
