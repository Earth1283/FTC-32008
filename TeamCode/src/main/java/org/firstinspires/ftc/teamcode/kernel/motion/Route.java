package org.firstinspires.ftc.teamcode.kernel.motion;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.kernel.errors.KernelPanic;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class Route {
    private Route() {}

    public static Path line(Pose from, Pose to) {
        return segment(from, to);
    }

    public static Path segment(Pose from, Pose to, Pose... controlPoints) {
        Pose start = new Pose(from.getX(), from.getY());
        Pose end = new Pose(to.getX(), to.getY());
        Path path;
        if (controlPoints.length == 0) {
            path = new Path(new BezierLine(start, end));
        } else {
            List<Pose> points = new ArrayList<>(controlPoints.length + 2);
            points.add(start);
            points.addAll(Arrays.asList(controlPoints));
            points.add(end);
            path = new Path(new BezierCurve(points));
        }
        path.setLinearHeadingInterpolation(from.getHeading(), to.getHeading());
        return path;
    }

    public static Path[] through(Pose... waypoints) {
        if (waypoints.length < 2) throw new KernelPanic("Route.through() requires at least two poses.");
        Path[] legs = new Path[waypoints.length - 1];
        for (int i = 0; i < legs.length; i++) legs[i] = line(waypoints[i], waypoints[i + 1]);
        return legs;
    }
}
