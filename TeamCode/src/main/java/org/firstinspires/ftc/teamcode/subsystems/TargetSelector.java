package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Configurable
public final class TargetSelector {
    public static double RED_X = 58.0, RED_LEFT_Y = 88.0, RED_RIGHT_Y = 54.0;
    public static double BLUE_X = 84.0, BLUE_LEFT_Y = 54.0, BLUE_RIGHT_Y = 88.0;
    public static double MID = 70.75;

    private TargetSelector() {}

    public static Pose2D getTarget(Pose2D current, boolean isRed) {
        boolean highY = current.getY(DistanceUnit.INCH) > MID;
        return getTargetForSide(isRed, isRed == highY);
    }

    public static Pose2D getTargetForSide(boolean isRed, boolean leftTarget) {
        double targetX = isRed ? RED_X : BLUE_X;
        double targetY = isRed
                ? (leftTarget ? RED_LEFT_Y : RED_RIGHT_Y)
                : (leftTarget ? BLUE_LEFT_Y : BLUE_RIGHT_Y);
        return new Pose2D(DistanceUnit.INCH, targetX, targetY, AngleUnit.RADIANS, 0.0);
    }
}
