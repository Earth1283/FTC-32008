package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.DOWN_SHOOTER;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.UP_SHOOTER;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.autoTurretAim;

import java.util.List;
import java.util.Locale;

/**
 * Tune shot speed, aim, and panel position against a measured target.
 * Compare both distance calculations and press gamepad 2 X once the flywheel speed stabilizes.
 */
@Configurable
@TeleOp(name = "Shooter Distance Test", group = "Test")
public final class ShooterDistanceTest extends LinearOpMode {
    // Starting field pose in inches and degrees, read at START.
    public static double START_X_IN = 71.0;
    public static double START_Y_IN = 9.0;
    public static double START_H_DEG = 90.0;

    // Set both targets to the same field coordinates to test a single target.
    public static double LOW_TARGET_X_IN = 58.0;
    public static double LOW_TARGET_Y_IN = 55.0;
    public static double HIGH_TARGET_X_IN = 58.0;
    public static double HIGH_TARGET_Y_IN = 85.0;
    public static double MID_Y_IN = 70.75;
    public static double TARGET_SWITCH_BUFFER_IN = 2.0;

    // Initial settings for this test. Fine-tune with the gamepad and press X to record.
    public static double FLYWHEEL_START_TICKS_PER_SECOND = 1120.0;
    public static double PANEL_START_POSITION = 0.250;
    private static final double MAX_TEST_TICKS_PER_SECOND = 2800.0;
    private static final double FLYWHEEL_STEP_TICKS_PER_SECOND = 10.0;
    private static final double AIM_STEP_DEG = 2.0;
    private static final double PANEL_STEP = 0.01;
    private static final double TRIGGER_THRESHOLD = 0.05;
    private static final double TELEMETRY_INTERVAL_MS = 100.0;

    private int sampleCount;
    private String lastSample = "No samples yet";
    private String recordStatus = "Press gamepad 2 X when the flywheel speed stabilizes";

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain();
        Intake intake = new Intake();
        Shooter shooter = new Shooter();
        drivetrain.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        // Read the power commands for both flywheels in telemetry.
        DcMotorEx upFlywheel = hardwareMap.get(DcMotorEx.class, UP_SHOOTER);
        DcMotorEx downFlywheel = hardwareMap.get(DcMotorEx.class, DOWN_SHOOTER);
        // Read every analog input in the active robot configuration.
        List<AnalogInput> analogInputs = hardwareMap.getAll(AnalogInput.class);
        autoTurretAim turret = new autoTurretAim(hardwareMap);
        JoinedTelemetry display = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        display.addLine("Set the starting pose and test targets in ShooterDistanceTest in Panels.");
        display.addLine("Gamepad 1: RT collects, LT ejects, LB toggles the flywheel, and RB feeds.");
        display.addLine("Gamepad 2: D-pad up/down adjusts speed, left/right trims aim, Y/A adjusts the panel, and X records.");
        display.update();
        intake.shovelUp();
        waitForStart();
        if (isStopRequested()) return;

        double startX = START_X_IN;
        double startY = START_Y_IN;
        double startHeading = START_H_DEG;
        if (!Double.isFinite(startX) || !Double.isFinite(startY) || !Double.isFinite(startHeading)) {
            display.addLine("Invalid starting pose. Check START_X_IN, START_Y_IN, and START_H_DEG.");
            display.update();
            return;
        }
        drivetrain.setPosition(new Pose2D(DistanceUnit.INCH, startX, startY,
                AngleUnit.DEGREES, startHeading));

        boolean highSide = startY > MID_Y_IN;
        boolean flywheelOn = false;
        double flywheelTarget = FLYWHEEL_START_TICKS_PER_SECOND;
        double panelPosition = PANEL_START_POSITION;
        double lowAimTrim = 0.0;
        double highAimTrim = 0.0;
        ElapsedTime telemetryTimer = new ElapsedTime();

        try {
            while (opModeIsActive()) {
                // Refresh Pinpoint each loop so the distance tracks robot movement.
                drivetrain.update();
                drivetrain.drive(gamepad1, 1.0);
                Pose2D current = drivetrain.getPosition();
                double x = current.getX(DistanceUnit.INCH);
                double y = current.getY(DistanceUnit.INCH);
                double heading = current.getHeading(AngleUnit.DEGREES);
                DeviceStatus status = drivetrain.getLocalizationStatus();
                boolean poseReady = status == DeviceStatus.READY
                        && Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(heading);

                // Switch targets after crossing the midfield buffer and remember a trim for each target.
                double buffer = Math.max(0.0, TARGET_SWITCH_BUFFER_IN);
                double midY = MID_Y_IN;
                if (poseReady && y > midY + buffer) highSide = true;
                else if (poseReady && y <= midY - buffer) highSide = false;

                double trimStep = 0.0;
                if (gamepad2.dpadLeftWasPressed()) trimStep -= AIM_STEP_DEG;
                if (gamepad2.dpadRightWasPressed()) trimStep += AIM_STEP_DEG;
                if (highSide) highAimTrim += trimStep;
                else lowAimTrim += trimStep;
                double aimTrim = highSide ? highAimTrim : lowAimTrim;
                double targetX = highSide ? HIGH_TARGET_X_IN : LOW_TARGET_X_IN;
                double targetY = highSide ? HIGH_TARGET_Y_IN : LOW_TARGET_Y_IN;
                Pose2D target = new Pose2D(DistanceUnit.INCH, targetX, targetY, AngleUnit.DEGREES, 0.0);

                // Aim also measures the distance from the turret pivot. Hold the turret while localization is unavailable.
                turret.setEnabled(poseReady);
                double distanceInches = Double.NaN;
                if (poseReady) {
                    turret.aimAt(current, target, aimTrim);
                    distanceInches = turret.getDistanceInches();
                }

                // Adjust the flywheel and panel independently while testing.
                if (gamepad1.leftBumperWasPressed()) flywheelOn = !flywheelOn;
                boolean up1 = gamepad1.dpadUpWasPressed();
                boolean up2 = gamepad2.dpadUpWasPressed();
                boolean down1 = gamepad1.dpadDownWasPressed();
                boolean down2 = gamepad2.dpadDownWasPressed();
                if (up1 || up2) flywheelTarget += FLYWHEEL_STEP_TICKS_PER_SECOND;
                if (down1 || down2) flywheelTarget -= FLYWHEEL_STEP_TICKS_PER_SECOND;
                flywheelTarget = Range.clip(flywheelTarget, 0.0, MAX_TEST_TICKS_PER_SECOND);

                boolean showTelemetry = telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS;
                boolean runFlywheel = flywheelOn && Double.isFinite(flywheelTarget) && flywheelTarget > 0.0;
                double measuredVelocity = Double.NaN;
                if (runFlywheel || showTelemetry) measuredVelocity = shooter.getPollenVelocity();
                if (runFlywheel && Double.isFinite(measuredVelocity)) {
                    shooter.setPollenMotorVelocity(flywheelTarget, measuredVelocity);
                } else {
                    shooter.stop();
                    runFlywheel = false;
                }

                if (gamepad2.yWasPressed()) panelPosition += PANEL_STEP;
                if (gamepad2.aWasPressed()) panelPosition -= PANEL_STEP;
                double appliedPanel = shooter.setPanel(panelPosition);
                if (Double.isFinite(appliedPanel)) panelPosition = appliedPanel;

                // Triggers collect or eject. Holding the right bumper opens the gate and feeds.
                if (gamepad1.right_trigger > TRIGGER_THRESHOLD) intake.intakeIn();
                else if (gamepad1.left_trigger > TRIGGER_THRESHOLD) intake.intakeOut();
                else if (gamepad1.right_bumper) intake.intakeFeedWithGateDelay();
                else intake.intakeStop();
                if (gamepad1.yWasPressed()) intake.shovelUp();
                if (gamepad1.aWasPressed()) intake.shovelDown();

                // Check for a press every loop, even between telemetry updates.
                if (gamepad2.xWasPressed()) {
                    recordSample(target, distanceInches, runFlywheel, flywheelTarget, measuredVelocity, appliedPanel);
                }
                if (!showTelemetry) continue;
                display.addData("Pinpoint", status);
                display.addData("Robot X / Y / heading", "%.1f / %.1f / %.1f", x, y, heading);
                display.addData("Active target X / Y (in)", "%.1f / %.1f", targetX, targetY);
                display.addData("Turret trim (deg)", "%+.1f", aimTrim);
                display.addData("Flywheel", runFlywheel ? "On" : "Off");
                display.addData("Target speed (ticks/s)", flywheelTarget);
                display.addData("Measured speed (ticks/s)", measuredVelocity);
                // Show the gains and signed encoder readings to help check both motor directions.
                display.addData("Flywheel KP / KV / KS", "%.8f / %.8f / %.8f",
                        Shooter.POLLEN_KP, Shooter.POLLEN_KV, Shooter.POLLEN_KS);
                display.addData("Upper flywheel power / speed (ticks/s)", "%.6f / %+.1f",
                        upFlywheel.getPower(), shooter.getUpVelocity());
                display.addData("Lower flywheel power / speed (ticks/s)", "%.6f / %+.1f",
                        downFlywheel.getPower(), shooter.getDownVelocity());
                // Read voltages only when updating telemetry.
                if (analogInputs.isEmpty()) {
                    display.addData("Analog inputs", "None in the active robot configuration");
                } else {
                    for (AnalogInput analogInput : analogInputs) {
                        display.addData("Analog " + hardwareMap.getNamesOf(analogInput),
                                "%.3f V", analogInput.getVoltage());
                    }
                }
                display.addData("Panel position command", "%.3f", appliedPanel);
                if (poseReady) {
                    turret.addTelemetry(display);
                    double testDistanceInches = calculateTestDistanceInches(current, target);
                    display.addData("Distance comparison (in)", "Turret %.3f / test %.3f",
                            distanceInches, testDistanceInches);
                    double difference = Math.abs(testDistanceInches - distanceInches);
                    if (Double.isFinite(difference)) {
                        display.addData("Distance difference (in)", "%.4f", difference);
                    } else {
                        display.addData("Distance difference (in)", "Invalid distance");
                    }
                } else {
                    display.addData("Distance comparison (in)", "Waiting for localization");
                }
                display.addData("Sample status", recordStatus);
                display.addData("Latest sample", lastSample);
                display.update();
                telemetryTimer.reset();
            }
        } finally {
            // Stop the wheels and flywheel, close the gate, and disable turret output.
            drivetrain.drive(gamepad1, 0.0);
            shooter.stop();
            intake.intakeStop();
            turret.setEnabled(false);
        }
    }

    /** Find the turret pivot in field coordinates and measure its planar distance to the target. */
    private static double calculateTestDistanceInches(Pose2D robot, Pose2D target) {
        // Both calculations share the measured offsets. Forward and left are positive.
        double forwardOffset = autoTurretAim.POLLEN_SHOOTER_DRIVETRAIN_OFFSET_MM / 25.4;
        double leftOffset = autoTurretAim.TURRET_LEFT_OFFSET_MM / 25.4;
        double heading = robot.getHeading(AngleUnit.RADIANS);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // Rotate the chassis offsets into field coordinates and add them to the Pinpoint pose.
        double turretX = robot.getX(DistanceUnit.INCH) + forwardOffset * cos - leftOffset * sin;
        double turretY = robot.getY(DistanceUnit.INCH) + forwardOffset * sin + leftOffset * cos;
        double dx = target.getX(DistanceUnit.INCH) - turretX;
        double dy = target.getY(DistanceUnit.INCH) - turretY;
        return Math.hypot(dx, dy);
    }

    private void recordSample(Pose2D target, double distanceInches, boolean flywheelRunning,
                              double targetVelocity, double measuredVelocity, double panelPosition) {
        // Only record a sample when the pose, flywheel speed, and panel command are valid.
        if (!Double.isFinite(distanceInches) || !flywheelRunning
                || !Double.isFinite(measuredVelocity) || !Double.isFinite(panelPosition)) {
            recordStatus = "Check Pinpoint, the flywheel switch, and the panel command";
            return;
        }

        double targetX = target.getX(DistanceUnit.INCH);
        double targetY = target.getY(DistanceUnit.INCH);
        sampleCount++;
        lastSample = String.format(Locale.US,
                "#%d | Distance %.2f in | Speed %.0f / %.0f ticks/s | Panel %.3f | Target (%.1f, %.1f)",
                sampleCount, distanceInches, targetVelocity, measuredVelocity, panelPosition, targetX, targetY);
        recordStatus = "Sample saved. Record hit counts separately.";

        // Show the latest sample on the Driver Station and log every sample on the Robot Controller.
        RobotLog.ii("ShooterDistanceTest",
                "sample=%d distance_in=%.2f target_ticks_per_s=%.0f actual_ticks_per_s=%.0f panel=%.3f target_x_in=%.1f target_y_in=%.1f",
                sampleCount, distanceInches, targetVelocity, measuredVelocity, panelPosition, targetX, targetY);
    }
}
