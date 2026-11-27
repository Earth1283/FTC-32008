package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.commands.FollowPathCommand.fromPose;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_PARK_E;
import static org.firstinspires.ftc.teamcode.constants.AutoConst.RED_PARK_S;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Flower - Park Only")
public class FLOWER_0TIP extends CompetitionAuto {
    @Override
    protected Pose startingPose() {
        return RED_PARK_S;
    }

    @Override
    protected Command buildRoute() {
        return new SequentialCommandGroup(

                // Drive straight to the parking point and interpolate between the start and end headings.
                fromPose(robot.motion, RED_PARK_S, RED_PARK_E),
                new InstantCommand(this::requestOpModeStop)
        );
    }
}
