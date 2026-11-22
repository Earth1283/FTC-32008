package org.firstinspires.ftc.teamcode.kernel;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kernel.errors.InitFailedPanic;
import org.firstinspires.ftc.teamcode.kernel.errors.KernelPanic;
import org.firstinspires.ftc.teamcode.kernel.errors.OpModeStoppedException;
import org.firstinspires.ftc.teamcode.kernel.errors.ProfileMisconfiguredPanic;
import org.firstinspires.ftc.teamcode.kernel.errors.SubsystemUnavailableException;
import org.firstinspires.ftc.teamcode.kernel.errors.TickReentrancyPanic;
import org.firstinspires.ftc.teamcode.kernel.errors.WrongOpModeTypePanic;
import org.firstinspires.ftc.teamcode.kernel.events.Event;
import org.firstinspires.ftc.teamcode.kernel.events.EventBus;
import org.firstinspires.ftc.teamcode.kernel.events.Subscription;
import org.firstinspires.ftc.teamcode.kernel.init.InitLog;
import org.firstinspires.ftc.teamcode.kernel.init.InitResult;
import org.firstinspires.ftc.teamcode.kernel.init.InitTask;
import org.firstinspires.ftc.teamcode.kernel.input.Input;
import org.firstinspires.ftc.teamcode.kernel.input.Pad;
import org.firstinspires.ftc.teamcode.kernel.input.SdkPadSource;
import org.firstinspires.ftc.teamcode.kernel.motion.Motion;
import org.firstinspires.ftc.teamcode.kernel.motion.PedroMotion;
import org.firstinspires.ftc.teamcode.kernel.profiles.Profiles;
import org.firstinspires.ftc.teamcode.kernel.profiles.RobotProfile;
import org.firstinspires.ftc.teamcode.kernel.subsystems.Claw;
import org.firstinspires.ftc.teamcode.kernel.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.kernel.subsystems.Intake;
import org.firstinspires.ftc.teamcode.kernel.subsystems.Lift;
import org.firstinspires.ftc.teamcode.kernel.subsystems.Rangefinder;
import org.firstinspires.ftc.teamcode.kernel.subsystems.Turret;
import org.firstinspires.ftc.teamcode.kernel.telemetry.KernelTelemetry;
import org.firstinspires.ftc.teamcode.kernel.telemetry.SdkTelemetrySink;
import org.firstinspires.ftc.teamcode.kernel.time.LoopTimer;
import org.firstinspires.ftc.teamcode.kernel.time.MatchClock;
import org.firstinspires.ftc.teamcode.kernel.time.Scheduler;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

public final class Robot {
    public final Motion motion;
    public final Claw claw;
    public final Lift lift;
    public final Intake intake;
    public final Flywheel flywheel;
    public final Turret turret;
    public final Rangefinder rangefinder;

    public final Input input;
    public final Pad gp1;
    public final Pad gp2;
    public final KernelTelemetry telemetry;

    private final OpMode opMode;
    private final Kernel kernel;
    private final EventBus events = new EventBus();
    private final PedroMotion pedroMotion;
    private final LongSupplier nanoClock = System::nanoTime;
    private final Scheduler scheduler = new Scheduler(nanoClock);
    private final LoopTimer loopTimer = new LoopTimer(nanoClock);
    private final InitLog initLog = new InitLog(nanoClock);
    private final MatchClock matchClock;
    private final StatusPanel statusPanel;
    private final boolean indicateAllianceOnGamepads;
    private final boolean autonomous;
    private final MemberBinder members;
    // The SDK holds lifecycle listeners weakly; this field keeps ours alive.
    private final OpModeManagerNotifier.Notifications lifecycle;

    private Alliance alliance = Alliance.BLUE;
    private boolean ticking;
    private volatile boolean stopped;
    private volatile boolean started;

    public Robot() {
        this(ActiveOpMode.require());
    }

    public Robot(OpMode opMode) {
        this(opMode, Profiles.ACTIVE, opMode.telemetry, true);
    }

    Robot(KernelOpMode opMode, RobotProfile profile, Telemetry display) {
        this(opMode, profile, display, false);
    }

    private Robot(OpMode opMode, RobotProfile profile, Telemetry display, boolean observeSdkLifecycle) {
        this.opMode = opMode;
        indicateAllianceOnGamepads = profile.indicateAllianceOnGamepads();

        kernel = new Kernel(opMode.hardwareMap, profile.name(), events, nanoClock);

        Follower follower = profile.follower(opMode.hardwareMap);
        pedroMotion = follower == null ? null : new PedroMotion(follower, events, profile.motionTuning(), nanoClock);
        motion = pedroMotion;

        claw = install(Claw.class, profile.claw(kernel));
        lift = install(Lift.class, profile.lift(kernel));
        intake = install(Intake.class, profile.intake(kernel));
        flywheel = install(Flywheel.class, profile.flywheel(kernel));
        turret = install(Turret.class, profile.turret(kernel));
        rangefinder = install(Rangefinder.class, profile.rangefinder(kernel));
        for (Map.Entry<Class<? extends Subsystem>, Subsystem> custom : profile.custom(kernel).entrySet()) {
            installCustom(custom.getKey(), custom.getValue());
        }

        input = new Input(new SdkPadSource(() -> opMode.gamepad1), new SdkPadSource(() -> opMode.gamepad2),
                profile.stickDeadband(), profile.stickCurve(), nanoClock);
        gp1 = input.gp1;
        gp2 = input.gp2;

        autonomous = isAutonomous(opMode);
        matchClock = autonomous ? MatchClock.autonomous(events, nanoClock) : MatchClock.teleOp(events, nanoClock);
        telemetry = new KernelTelemetry(new SdkTelemetrySink(display), nanoClock);
        statusPanel = new StatusPanel(kernel, motion, input, initLog, matchClock, loopTimer, autonomous);
        members = new MemberBinder(telemetry, scheduler);

        if (profile.manageBulkCaching()) kernel.takeOverBulkCaching();
        alliance(Alliance.BLUE);
        if (!autonomous && motion != null) PoseStore.resumeInto(this);
        PlayAs playAs = opMode.getClass().getAnnotation(PlayAs.class);
        if (playAs != null) alliance(playAs.value());

        lifecycle = observeSdkLifecycle ? new ShutdownOnStop() : null;
        bind(opMode);
        runBootTasks(profile);
        if (lifecycle != null) ActiveOpMode.manager().registerListener(lifecycle);
    }

    private void runBootTasks(RobotProfile profile) {
        for (InitTask task : BootChecks.forRobot(kernel, motion, profile)) initLog.run(task);
        try {
            initLog.throwIfCriticalFailure();
        } catch (InitFailedPanic failure) {
            showStatus();
            panic(failure);
            throw failure;
        }
        showStatus();
    }

    private <T extends Subsystem> T install(Class<T> type, T subsystem) {
        if (subsystem == null) return Missing.subsystem(type, kernel.profileName());
        kernel.register(type, subsystem);
        return subsystem;
    }

    @SuppressWarnings("unchecked")
    private <T extends Subsystem> void installCustom(Class<? extends Subsystem> type, Subsystem subsystem) {
        kernel.register((Class<T>) type, (T) subsystem);
    }

    private static boolean isAutonomous(OpMode opMode) {
        return opMode.getClass().isAnnotationPresent(Autonomous.class);
    }

    public void tick() {
        tick(null);
    }

    void tick(Runnable controlLoop) {
        if (ticking) throw new TickReentrancyPanic();
        if (stopped) return;
        ticking = true;
        try {
            loopTimer.mark();
            kernel.clearBulkCaches();
            input.sample();
            if (started) {
                members.start();
                input.fireBindings();
            }
            if (pedroMotion != null) pedroMotion.update();
            for (Subsystem subsystem : kernel.subsystems()) subsystem.update();
            if (controlLoop != null) controlLoop.run();
            scheduler.update();
            matchClock.update();
            events.dispatch();
            if (telemetry.isDue()) showStatus();
        } catch (RuntimeException e) {
            panic(e);
            throw e;
        } finally {
            ticking = false;
        }
    }

    private void panic(RuntimeException cause) {
        stopOutputs();
        String label = cause instanceof KernelPanic ? "KERNEL PANIC: " : "KERNEL PANIC (" + cause.getClass().getSimpleName() + "): ";
        RobotLog.setGlobalErrorMsg(label + cause.getMessage());
    }

    private void showStatus() {
        telemetry.publish(statusPanel.render(alliance));
    }

    // Use this in place of waitForStart(): the init screen keeps updating and gamepad state keeps sampling.
    // Key bindings stay disarmed until start, so a stray press during init can't move a mechanism.
    public void awaitStart() throws OpModeStoppedException {
        awaitStart(null);
    }

    void awaitStart(Runnable initLoop) throws OpModeStoppedException {
        LinearOpMode linear = requireLinear("awaitStart");
        while (!linear.isStarted()) {
            if (linear.isStopRequested() || stopped) throw new OpModeStoppedException();
            tick(initLoop);
        }
    }

    public void waitUntil(BooleanSupplier condition) throws OpModeStoppedException {
        LinearOpMode linear = requireLinear("waitUntil");
        while (!condition.getAsBoolean()) {
            if (linear.isStopRequested() || stopped) throw new OpModeStoppedException();
            tick();
        }
    }

    public void sleep(long millis) throws OpModeStoppedException {
        requireLinear("sleep");
        long end = nanoClock.getAsLong() + millis * 1_000_000L;
        waitUntil(() -> nanoClock.getAsLong() >= end);
    }

    public void await(Awaitable... things) throws OpModeStoppedException {
        requireLinear("await");
        waitUntil(() -> allSettled(things));
    }

    private static boolean allSettled(Awaitable[] things) {
        for (Awaitable thing : things) if (!thing.isSettled()) return false;
        return true;
    }

    private LinearOpMode requireLinear(String method) {
        if (ticking) throw new TickReentrancyPanic();
        if (!(opMode instanceof LinearOpMode)) throw new WrongOpModeTypePanic(method, opMode.getClass());
        return (LinearOpMode) opMode;
    }

    // The OpMode is bound automatically; call this for helper objects with annotated members.
    public void bind(Object target) {
        input.bind(target);
        members.bind(target);
    }

    public <T> Subscription on(Event<T> event, Consumer<? super T> handler) {
        return events.on(event, handler);
    }

    public Subscription on(Event<?> event, Runnable handler) {
        return events.on(event, handler);
    }

    public Subscription after(long millis, Runnable action) {
        return scheduler.after(millis, action);
    }

    public Subscription every(long millis, Runnable action) {
        return scheduler.every(millis, action);
    }

    public double matchSecondsRemaining() {
        return matchClock.secondsRemaining();
    }

    public InitResult init(String name, Supplier<InitResult> body) {
        return initLog.run(InitTask.of(name, body));
    }

    public <T extends Subsystem> T get(Class<T> type) throws SubsystemUnavailableException {
        return kernel.get(type);
    }

    public <T extends Subsystem> T require(Class<T> type) {
        try {
            return get(type);
        } catch (SubsystemUnavailableException absent) {
            throw new ProfileMisconfiguredPanic(absent.getMessage());
        }
    }

    public Alliance alliance() {
        return alliance;
    }

    public void alliance(Alliance alliance) {
        this.alliance = alliance;
        if (pedroMotion != null) pedroMotion.setDriverForwardHeading(alliance.heading(0));
        if (indicateAllianceOnGamepads) showAllianceColor();
    }

    private void showAllianceColor() {
        double red = alliance == Alliance.RED ? 1 : 0;
        double blue = alliance == Alliance.BLUE ? 1 : 0;
        gp1.led(red, 0, blue);
        gp2.led(red, 0, blue);
    }

    public boolean has(Class<? extends Subsystem> type) {
        try {
            kernel.get(type);
            return true;
        } catch (SubsystemUnavailableException notInstalled) {
            return false;
        }
    }

    private void stopOutputs() {
        if (pedroMotion != null) stop(pedroMotion);
        for (Subsystem subsystem : kernel.subsystems()) {
            stop(subsystem);
        }
        events.clear();
        scheduler.clear();
    }

    private void stop(Subsystem subsystem) {
        try {
            subsystem.stop();
        } catch (RuntimeException failure) {
            RobotLog.ee("Kernel", failure, "Subsystem shutdown failed");
        }
    }

    void start() {
        matchClock.start();
        started = true;
    }

    void shutdown() {
        if (stopped) return;
        stopped = true;
        try {
            if (started && autonomous && motion != null) PoseStore.save(motion.pose(), alliance);
        } finally {
            stopOutputs();
        }
    }

    private final class ShutdownOnStop implements OpModeManagerNotifier.Notifications {
        @Override
        public void onOpModePreInit(OpMode ignored) {}

        @Override
        public void onOpModePreStart(OpMode startingOpMode) {
            if (startingOpMode != opMode) return;
            start();
        }

        @Override
        public void onOpModePostStop(OpMode stoppedOpMode) {
            if (stoppedOpMode != opMode) return;
            shutdown();
        }
    }
}
