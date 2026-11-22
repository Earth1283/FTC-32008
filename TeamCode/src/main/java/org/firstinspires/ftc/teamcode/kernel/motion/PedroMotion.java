package org.firstinspires.ftc.teamcode.kernel.motion;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.kernel.Angles;
import org.firstinspires.ftc.teamcode.kernel.Subsystem;
import org.firstinspires.ftc.teamcode.kernel.events.EventBus;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;
import java.util.function.LongSupplier;

public final class PedroMotion implements Motion, Subsystem {
    private final FollowerPort follower;
    private final EventBus events;
    private final MotionTuning tuning;
    private final LongSupplier nanoClock;

    private final Deque<Path> pending = new ArrayDeque<>();
    private Path current;
    private Mode mode = Mode.IDLE;

    private double requestedForward, requestedLeft, requestedTurn;
    private double driverForwardHeading;
    private double speedScale = 1.0;

    private Pose aimTarget;
    private double previousAimError;
    private long previousAimNanos;
    private double turnTarget;

    public PedroMotion(Follower follower, EventBus events, MotionTuning tuning, LongSupplier nanoClock) {
        this(new PedroFollowerPort(follower), events, tuning, nanoClock);
    }

    PedroMotion(FollowerPort follower, EventBus events, MotionTuning tuning, LongSupplier nanoClock) {
        this.follower = follower;
        this.events = events;
        this.tuning = tuning;
        this.nanoClock = nanoClock;
    }

    @Override
    public void drive(double forward, double left, double counterClockwise) {
        boolean autonomousMotion = mode == Mode.FOLLOW || mode == Mode.TURN || mode == Mode.HOLD;
        if (autonomousMotion && !exceedsTakeover(forward, left, counterClockwise)) return;
        if (mode != Mode.DRIVE) {
            pending.clear();
            current = null;
            mode = Mode.DRIVE;
        }
        requestedForward = forward * speedScale;
        requestedLeft = left * speedScale;
        requestedTurn = counterClockwise * speedScale;
    }

    @Override
    public void setSpeedScale(double scale) {
        speedScale = scale;
    }

    private boolean exceedsTakeover(double... inputs) {
        for (double input : inputs) if (Math.abs(input) > tuning.driverTakeoverThreshold) return true;
        return false;
    }

    @Override
    public Motion follow(Path path, boolean holdEnd, double maxPower) {
        pending.clear();
        start(path, holdEnd, maxPower);
        return this;
    }

    @Override
    public Motion queue(Path... paths) {
        pending.addAll(Arrays.asList(paths));
        if (mode != Mode.FOLLOW && !pending.isEmpty()) start(pending.poll(), true, 1.0);
        return this;
    }

    @Override
    public Motion goTo(Pose target) {
        Pose here = follower.pose();
        if (Math.hypot(target.getX() - here.getX(), target.getY() - here.getY()) < tuning.goToMinDistance) {
            return turnTo(target.getHeading());
        }
        return follow(Route.line(here, target));
    }

    @Override
    public Motion turnBy(double radians) {
        return turnTo(Angles.wrap(follower.pose().getHeading() + radians));
    }

    private void start(Path path, boolean holdEnd, double maxPower) {
        current = path;
        mode = Mode.FOLLOW;
        follower.follow(path, holdEnd, maxPower);
    }

    @Override
    public void aimTo(Pose target) {
        boolean targetChanged = aimTarget == null || aimTarget.getX() != target.getX() || aimTarget.getY() != target.getY();
        aimTarget = target;
        if (targetChanged) previousAimNanos = 0;
        if (targetChanged && (mode == Mode.IDLE || mode == Mode.HOLD)) holdFacing(target);
    }

    @Override
    public void stopAiming() {
        aimTarget = null;
    }

    @Override
    public boolean isAiming() {
        return aimTarget != null;
    }

    private void holdFacing(Pose target) {
        Pose pose = follower.pose();
        follower.hold(new Pose(pose.getX(), pose.getY(), Angles.toward(pose.getX(), pose.getY(), target.getX(), target.getY())));
        mode = Mode.HOLD;
    }

    @Override
    public Motion turnTo(double heading) {
        pending.clear();
        current = null;
        turnTarget = heading;
        Pose pose = follower.pose();
        follower.hold(new Pose(pose.getX(), pose.getY(), heading));
        mode = Mode.TURN;
        return this;
    }

    @Override
    public Motion hold() {
        pending.clear();
        current = null;
        follower.hold(follower.pose());
        mode = Mode.HOLD;
        return this;
    }

    @Override
    public void cancel() {
        pending.clear();
        current = null;
        aimTarget = null;
        follower.stop();
        mode = Mode.IDLE;
    }

    @Override
    public void stop() {
        cancel();
    }

    @Override
    public Pose pose() {
        return follower.pose();
    }

    @Override
    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    @Override
    public void resetDriverForward() {
        driverForwardHeading = follower.pose().getHeading();
    }

    public void setDriverForwardHeading(double heading) {
        driverForwardHeading = heading;
    }

    @Override
    public boolean isBusy() {
        return mode == Mode.FOLLOW || mode == Mode.TURN;
    }

    @Override
    public boolean isStuck() {
        return mode == Mode.FOLLOW && follower.isStuck();
    }

    @Override
    public int queued() {
        return pending.size();
    }

    @Override
    public Mode mode() {
        return mode;
    }

    @Override
    public void update() {
        if (mode == Mode.DRIVE) applyDriverInput();
        if (mode == Mode.TURN) checkTurnDone();
        follower.update();
        if (mode == Mode.FOLLOW && !follower.following()) finishPath();
    }

    private void applyDriverInput() {
        Pose pose = follower.pose();
        double forward = requestedForward;
        double left = requestedLeft;
        if (tuning.fieldCentric) {
            double theta = pose.getHeading() - driverForwardHeading;
            forward = requestedForward * Math.cos(theta) + requestedLeft * Math.sin(theta);
            left = -requestedForward * Math.sin(theta) + requestedLeft * Math.cos(theta);
        }
        double turn = aimTarget != null ? aimTurn(pose) : requestedTurn;
        follower.manual(forward, left, turn);
        requestedForward = requestedLeft = requestedTurn = 0;
    }

    private double aimTurn(Pose pose) {
        double desired = Angles.toward(pose.getX(), pose.getY(), aimTarget.getX(), aimTarget.getY());
        double error = Angles.error(pose.getHeading(), desired);
        long now = nanoClock.getAsLong();
        double derivative = previousAimNanos == 0 ? 0 : (error - previousAimError) / ((now - previousAimNanos) / 1e9);
        previousAimError = error;
        previousAimNanos = now;
        double turn = tuning.aimKp * error + tuning.aimKd * derivative;
        return Math.max(-tuning.aimMaxTurn, Math.min(tuning.aimMaxTurn, turn));
    }

    private void checkTurnDone() {
        if (Math.abs(Angles.error(follower.pose().getHeading(), turnTarget)) > tuning.turnToleranceRadians) return;
        mode = Mode.HOLD;
        events.emit(TURN_DONE, turnTarget);
    }

    private void finishPath() {
        Path finished = current;
        current = null;
        events.emit(PATH_DONE, finished);
        if (!pending.isEmpty()) {
            start(pending.poll(), true, 1.0);
            return;
        }
        events.emit(QUEUE_EMPTY);
        if (aimTarget != null) holdFacing(aimTarget);
        else mode = follower.holding() ? Mode.HOLD : Mode.IDLE;
    }
}
