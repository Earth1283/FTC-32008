package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.kernel.motion.Motion;
import org.firstinspires.ftc.teamcode.kernel.motion.Route;

import java.util.Arrays;
import java.util.function.Supplier;

public final class FollowPathCommand extends CommandBase {
    private final Motion motion;
    private final Supplier<Pose> startPose;
    private final Pose endPose;
    private final Pose[] controlPoints;
    private double maxPower = 1.0;

    public static FollowPathCommand fromCurrentPose(Motion motion, Pose... waypoints) {
        return new FollowPathCommand(motion, motion::pose, waypoints);
    }

    public static FollowPathCommand fromPose(Motion motion, Pose start, Pose... waypoints) {
        Pose fixedStart = start.copy();
        return new FollowPathCommand(motion, () -> fixedStart, waypoints);
    }

    private FollowPathCommand(Motion motion, Supplier<Pose> startPose, Pose[] waypoints) {
        this.motion = motion;
        this.startPose = startPose;
        endPose = waypoints[waypoints.length - 1].copy();
        controlPoints = Arrays.copyOf(waypoints, waypoints.length - 1);
        for (int i = 0; i < controlPoints.length; i++) {
            Pose point = controlPoints[i];
            controlPoints[i] = new Pose(point.getX(), point.getY());
        }
    }

    public FollowPathCommand setMaxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    @Override
    public void initialize() {
        Path path = Route.segment(startPose.get(), endPose, controlPoints);
        path.setBrakingStrength(1.0);
        motion.follow(path, true, maxPower);
    }

    @Override
    public boolean isFinished() {
        return motion.isSettled() || motion.isStuck();
    }

    @Override
    public void end(boolean interrupted) {
        // Pedro keeps following when a timed intake group ends.
    }

}
