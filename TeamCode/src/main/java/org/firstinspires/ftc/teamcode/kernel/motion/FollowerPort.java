package org.firstinspires.ftc.teamcode.kernel.motion;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

interface FollowerPort {
    void follow(Path path, boolean holdEnd, double maxPower);

    void hold(Pose pose);

    void manual(double forward, double left, double counterClockwise);

    void stop();

    void setPose(Pose pose);

    Pose pose();

    boolean following();

    boolean holding();

    boolean isStuck();

    void update();
}
