package org.firstinspires.ftc.teamcode.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.kernel.Alliance;
import org.firstinspires.ftc.teamcode.kernel.input.Key;
import org.firstinspires.ftc.teamcode.kernel.telemetry.Watch;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.TargetSelector;

import java.util.Locale;

@Configurable
public abstract class TurretTeleOpBase extends CompetitionTeleOp {
    public static boolean USE_CUSTOM_START_POSE = true;
    public static boolean CUSTOM_IS_RED = true;
    public static double START_X_IN = 71;
    public static double START_Y_IN = 9;
    public static double START_H_DEG = 90.0;
    // Zero points the driver's forward stick toward field +X.
    public static double HEADLESS_FORWARD_DEG = 0.0;
    public static double AIM_SWITCH_HYSTERESIS_IN = 2.0;
    public static double PANEL_START_POSITION = 0.50;

    private static final double TRIGGER_THRESHOLD = 0.05;
    private static final double FLYWHEEL_START_TICKS_PER_SECOND = 1400.0;
    private static final double FLYWHEEL_STEP_TICKS_PER_SECOND = 10.0;
    private static final double AIM_STEP_DEG = 2.0;
    private static final double PANEL_STEP = 0.01;

    private final boolean fieldOriented;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private boolean highY;
    private boolean flywheelOn;
    private double flywheelTarget = FLYWHEEL_START_TICKS_PER_SECOND;
    private double leftTargetTrim;
    private double rightTargetTrim;
    private double headlessForwardDeg;
    private double panelPosition;
    private double longestLoopMs;

    protected TurretTeleOpBase(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
    }

    @Override
    protected final void onInit() {
        super.onInit();
        intake.shovelUp();
    }

    @Watch("Drive")
    final String driveMode() {
        return fieldOriented ? "Field centric" : "Robot centric";
    }

    @Override
    protected final void onInitLoop() {
        robot.telemetry.line("Set starting pose and field-forward angle in Panels before START.");
    }

    @Override
    protected final void onStart() {
        boolean customPose = USE_CUSTOM_START_POSE;
        robot.alliance(customPose ? (CUSTOM_IS_RED ? Alliance.RED : Alliance.BLUE) : allianceFromAuto());
        Pose2D start = customPose
                ? new Pose2D(DistanceUnit.INCH, START_X_IN, START_Y_IN, AngleUnit.DEGREES, START_H_DEG)
                : startingPoseFromAuto();
        drivetrain.setPosition(start);
        highY = start.getY(DistanceUnit.INCH) > TargetSelector.MID;
        headlessForwardDeg = HEADLESS_FORWARD_DEG;
        panelPosition = PANEL_START_POSITION;
        loopTimer.reset();
    }

    @Override
    protected final void onLoop() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        longestLoopMs = Math.max(longestLoopMs, loopMs);
        boolean showTelemetry = robot.telemetry.isDue();

        Pose2D current = drivetrain.getPosition();
        DeviceStatus status = drivetrain.getLocalizationStatus();
        boolean poseReady = status == DeviceStatus.READY
                && Double.isFinite(current.getX(DistanceUnit.INCH))
                && Double.isFinite(current.getY(DistanceUnit.INCH))
                && Double.isFinite(current.getHeading(AngleUnit.DEGREES));

        drive(poseReady);
        updateTargetSide(current.getY(DistanceUnit.INCH), poseReady);
        boolean red = robot.alliance() == Alliance.RED;
        boolean leftTarget = red == highY;
        Pose2D target = TargetSelector.getTargetForSide(red, leftTarget);
        updateAim(current, target, poseReady, leftTarget);

        double measuredVelocity = updateFlywheel(showTelemetry);
        updateIntake(TRIGGER_THRESHOLD);
        double appliedPanelPosition = updatePanel();
        if (robot.gp1.takePress(Key.Y)) intake.shovelUp();
        if (robot.gp1.takePress(Key.A)) intake.shovelDown();

        if (showTelemetry) {
            publishTelemetry(current, status, target, leftTarget, measuredVelocity, appliedPanelPosition, loopMs);
        }
    }

    private void drive(boolean poseReady) {
        if (!fieldOriented) {
            driveRobotCentric();
        } else if (poseReady && Double.isFinite(headlessForwardDeg)) {
            driveFieldCentric(headlessForwardDeg);
        } else {
            drivetrain.stop();
        }
    }

    private void updateTargetSide(double y, boolean poseReady) {
        if (!poseReady) return;
        double mid = TargetSelector.MID;
        double hysteresis = Math.max(0.0, AIM_SWITCH_HYSTERESIS_IN);
        if (y > mid + hysteresis) highY = true;
        else if (y <= mid - hysteresis) highY = false;
    }

    private void updateAim(Pose2D current, Pose2D target, boolean poseReady, boolean leftTarget) {
        double trimStep = 0.0;
        if (robot.gp2.takePress(Key.DPAD_LEFT)) trimStep -= AIM_STEP_DEG;
        if (robot.gp2.takePress(Key.DPAD_RIGHT)) trimStep += AIM_STEP_DEG;
        if (leftTarget) leftTargetTrim += trimStep;
        else rightTargetTrim += trimStep;
        turret.setEnabled(poseReady);
        turret.aimAt(current, target, leftTarget ? leftTargetTrim : rightTargetTrim);
    }

    private double updateFlywheel(boolean showTelemetry) {
        if (robot.gp1.takePress(Key.LEFT_BUMPER)) flywheelOn = !flywheelOn;
        if (robot.gp1.takePress(Key.DPAD_UP) || robot.gp2.takePress(Key.DPAD_UP)) {
            flywheelTarget += FLYWHEEL_STEP_TICKS_PER_SECOND;
        }
        if (robot.gp1.takePress(Key.DPAD_DOWN) || robot.gp2.takePress(Key.DPAD_DOWN)) {
            flywheelTarget -= FLYWHEEL_STEP_TICKS_PER_SECOND;
        }
        flywheelTarget = Range.clip(flywheelTarget, 0.0, Math.max(0.0, Shooter.POLLEN_VELOCITY_MAX));
        boolean runFlywheel = flywheelOn && flywheelTarget > 0.0;
        double measuredVelocity = (runFlywheel || showTelemetry) ? shooter.getPollenVelocity() : 0.0;
        if (runFlywheel) shooter.setPollenMotorVelocity(flywheelTarget, measuredVelocity);
        else shooter.stop();
        return measuredVelocity;
    }

    private double updatePanel() {
        if (robot.gp2.takePress(Key.Y)) panelPosition += PANEL_STEP;
        if (robot.gp2.takePress(Key.A)) panelPosition -= PANEL_STEP;
        double appliedPosition = shooter.setPanel(panelPosition);
        if (Double.isFinite(appliedPosition)) panelPosition = appliedPosition;
        return appliedPosition;
    }

    private void publishTelemetry(Pose2D current, DeviceStatus status, Pose2D target, boolean leftTarget,
                                  double measuredVelocity, double appliedPanelPosition, double loopMs) {
        robot.telemetry.data("Loop last / max (ms)", String.format(Locale.US, "%.1f / %.1f", loopMs, longestLoopMs));
        robot.telemetry.data("Pinpoint", status);
        robot.telemetry.data("Pose X / Y / H", String.format(Locale.US, "%.1f / %.1f / %.1f",
                current.getX(DistanceUnit.INCH), current.getY(DistanceUnit.INCH), current.getHeading(AngleUnit.DEGREES)));
        robot.telemetry.data("Alliance / target", robot.alliance() + " / " + (leftTarget ? "LEFT" : "RIGHT"));
        robot.telemetry.data("Target X / Y (in)", String.format(Locale.US, "%.1f / %.1f",
                target.getX(DistanceUnit.INCH), target.getY(DistanceUnit.INCH)));
        robot.telemetry.data("Aim trim L / R (deg)", String.format(Locale.US, "%.1f / %.1f", leftTargetTrim, rightTargetTrim));
        robot.telemetry.data("Flywheel", flywheelOn ? "ON" : "OFF");
        robot.telemetry.data("Flywheel setpoint / actual (ticks/s)", String.format(Locale.US, "%.0f / %.0f",
                flywheelTarget, measuredVelocity));
        robot.telemetry.data("Panel", appliedPanelPosition);
    }
}
