package org.firstinspires.ftc.teamcode.kernel.motion;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;

final class PedroFollowerPort implements FollowerPort {
    private final Follower follower;
    private boolean holdAtEnd;

    PedroFollowerPort(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void follow(Path path, boolean holdEnd, double maxPower) {
        follower.breakFollowing();
        follower.setMaxPower(maxPower);
        double brakingStrength = path.getBrakingStrength();
        follower.followPath(new PathBuilder(follower).addPath(path).setBrakingStrength(brakingStrength).build(), holdEnd);
        holdAtEnd = holdEnd;
    }

    @Override
    public void hold(Pose pose) {
        follower.holdPoint(pose);
        holdAtEnd = true;
    }

    @Override
    public void manual(double forward, double left, double counterClockwise) {
        if (!follower.isTeleopDrive()) follower.startTeleopDrive();
        follower.setTeleOpDrive(forward, left, counterClockwise, true);
        holdAtEnd = false;
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        holdAtEnd = false;
    }

    @Override
    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    @Override
    public Pose pose() {
        return follower.getPose();
    }

    @Override
    public boolean following() {
        return follower.isBusy();
    }

    @Override
    public boolean isStuck() {
        return follower.isRobotStuck();
    }

    @Override
    public boolean holding() {
        return holdAtEnd && !following();
    }

    @Override
    public void update() {
        follower.update();
    }
}
