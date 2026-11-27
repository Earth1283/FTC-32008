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

@Autonomous(name = "Flower - Preload, Collect and Park")
public class FLOWER_1TIP_COOP extends CompetitionAuto {
    @Override
    protected Pose startingPose() {
        return RED_START_FLOWER;
    }

    @Override
    protected Command buildRoute() {
        return new SequentialCommandGroup(
                // Spin up the flywheel before feeding the preload.
                // Preload angle needs robot calibration for the current starting pose. Feeding is skipped until calibrated.
                fixedShot.prepare(1120, Double.NaN),
                new WaitCommand(SHOOTER_START_TIME),
                new WaitCommand(5000),
                fixedShot.feed(),
                new WaitCommand(EMPTY_MAGAZINE_TIME),
                new InstantCommand(intake::intakeStop),

                // Flower 1 needs about 163° at the current endpoint. Feeding is skipped until calibrated.
                fixedShot.prepare(1120, Double.NaN),
                // Drive to the Flower 1 collection area within the timed window.
                // The timer ends the command group while driving and intake run in parallel.
                new ParallelDeadlineGroup(
                        new WaitCommand(2500),
                        new SequentialCommandGroup(
                                new InstantCommand(intake::shovelDown),
                                fromPose(robot.motion, RED_START_FLOWER, RED_INTAKE_FLOWER_1_FROM_PRELOAD_1),
                                new InstantCommand(intake::intakeIn),
                                fromCurrentPose(robot.motion, RED_INTAKE_FLOWER_1_FROM_PRELOAD_2)
                        )
                ),
                new WaitCommand(FLOWER_RELOAD_TIME),
                new InstantCommand(intake::intakeStop),
                new WaitCommand(SHOOTER_READY_TIME),
                fixedShot.feed(),
                new WaitCommand(EMPTY_MAGAZINE_TIME),
                new InstantCommand(intake::intakeStop),
                new InstantCommand(shooter::stop),

                // Drive to the parking point and let the robot settle before handing off its pose.
                fromCurrentPose(robot.motion, RED_PARK_FROM_FLOWER),
                new WaitCommand(1000),

                new WaitCommand(100),
                new InstantCommand(this::requestOpModeStop)
        );
    }
}
