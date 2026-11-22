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

@Autonomous
public class FLOWER_0TIP_COOP extends CompetitionAuto {
    @Override
    protected Pose startingPose() {
        return RED_PARK_S;
    }

    @Override
    protected Command buildRoute() {
        return new SequentialCommandGroup(

                new WaitCommand(SHOOTER_START_TIME),
                new WaitCommand(3400),
                new InstantCommand(() -> intake.intakeFeed()),
                new WaitCommand(EMPTY_MAGAZINE_TIME + 200),
                new InstantCommand(() -> intake.intakeStop()),

                fromPose(robot.motion, RED_PARK_S, RED_PARK_M),
                fromCurrentPose(robot.motion, RED_PARK_E),

                new InstantCommand(() -> doneShooting = true),
                new InstantCommand(robot.motion::cancel)
        );
    }
}
