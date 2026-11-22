package org.firstinspires.ftc.teamcode.kernel;

import com.pedropathing.geometry.Pose;

// Static so it survives from one OpMode to the next (but not an app restart).
// The alliance travels with the pose: TeleOp's field-centric forward depends on it.
public final class PoseStore {
    private static Pose savedPose;
    private static Alliance savedAlliance;

    private PoseStore() {}

    static synchronized void save(Pose pose, Alliance alliance) {
        savedPose = pose.copy();
        savedAlliance = alliance;
    }

    static synchronized void resumeInto(Robot robot) {
        if (savedPose == null) return;
        robot.alliance(savedAlliance);
        robot.motion.setPose(savedPose.copy());
    }

    public static synchronized Pose peek() {
        return savedPose == null ? null : savedPose.copy();
    }

    public static synchronized Alliance peekAlliance() {
        return savedAlliance;
    }
}
