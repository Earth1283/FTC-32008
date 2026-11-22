package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_TURRET;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_TURRET;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.kernel.Kernel;
import org.firstinspires.ftc.teamcode.kernel.Subsystem;
import org.firstinspires.ftc.teamcode.kernel.telemetry.Report;

import java.util.Locale;

/** Turret angles are relative to the chassis. Clockwise is positive. */
@Configurable
public final class autoTurretAim implements Subsystem {
    public static double TOTAL_TRAVEL_DEG = 260.0;

    public static double LEFT_SERVO_AT_LEFT_130 = 0.97000;
    public static double LEFT_SERVO_AT_LEFT_90 = 0.87000;
    public static double LEFT_SERVO_AT_CENTER = 0.51000;
    public static double LEFT_SERVO_AT_RIGHT_90 = 0.18999;
    public static double LEFT_SERVO_AT_RIGHT_130 = 0.03999;
    public static double RIGHT_SERVO_AT_LEFT_130 = 0.98000;
    public static double RIGHT_SERVO_AT_LEFT_90 = 0.85000;
    public static double RIGHT_SERVO_AT_CENTER = 0.52000;
    public static double RIGHT_SERVO_AT_RIGHT_90 = 0.18999;
    public static double RIGHT_SERVO_AT_RIGHT_130 = 0.05999;

    // Offsets run from the localizer origin to the turret pivot. Forward and left are positive.
    public static double POLLEN_SHOOTER_DRIVETRAIN_OFFSET_MM = -33.6;
    public static double TURRET_LEFT_OFFSET_MM = 0.0;

    public static double MOTION_LEAD_TIME_SECONDS = 0.15;

    public static double AIM_DEADBAND_DEG = 0.0;

    private static final double MID_CALIBRATION_DEG = 90.0;
    private static final double MAX_CALIBRATION_DEG = 130.0;
    private static final double MM_PER_INCH = 25.4;
    private static final double MAX_SYNC_TEST_TRIM = 0.010;
    // Refresh both servo commands after a transient communication loss.
    private static final double SERVO_REFRESH_MS = 300.0;

    private final Servo leftTurret;
    private final Servo rightTurret;
    private final ElapsedTime servoTimer = new ElapsedTime();
    private boolean enabled;
    private boolean sendOnEnable;
    private double distanceInches = Double.NaN;
    private double requestedDeg = Double.NaN;
    private double limitedDeg = Double.NaN;
    // The turret has no position feedback.
    private double lastCommandedDeg = Double.NaN;
    private double lastLeftPosition = Double.NaN;
    private double lastRightPosition = Double.NaN;
    private double allowedHalfTravelDeg = Double.NaN;
    private String outputStatus = "Waiting for aim";

    public autoTurretAim(HardwareMap hardwareMap) {
        this(hardwareMap.get(Servo.class, LEFT_TURRET), hardwareMap.get(Servo.class, RIGHT_TURRET));
    }

    autoTurretAim(Kernel kernel) {
        this(kernel.device(Servo.class, LEFT_TURRET), kernel.device(Servo.class, RIGHT_TURRET));
    }

    private autoTurretAim(Servo leftTurret, Servo rightTurret) {
        this.leftTurret = leftTurret;
        this.rightTurret = rightTurret;
    }

    @Override
    public void stop() {
        setEnabled(false);
    }

    /** Disabling output leaves the servos at their last commanded position. */
    public void setEnabled(boolean enabled) {
        if (enabled && !this.enabled) {
            sendOnEnable = true;
        }
        this.enabled = enabled;
    }

    /** Returns whether the target is in range. Out-of-range tracking holds the previous command. */
    public boolean aimAt(Pose2D robot, Pose2D target, double trimDeg) {
        return aimAtWithLead(robot, target, trimDeg, 0.0, 0.0);
    }

    /** Field velocity is in inches per second. */
    public boolean aimAtWithMotionCompensation(Pose2D robot, Pose2D target, double trimDeg,
                                               double fieldVelocityX, double fieldVelocityY) {
        double leadX = fieldVelocityX * MOTION_LEAD_TIME_SECONDS;
        double leadY = fieldVelocityY * MOTION_LEAD_TIME_SECONDS;
        return aimAtWithLead(robot, target, trimDeg, leadX, leadY);
    }

    private boolean aimAtWithLead(Pose2D robot, Pose2D target, double trimDeg,
                                  double leadX, double leadY) {
        double heading = robot.getHeading(AngleUnit.RADIANS);
        double dx = target.getX(DistanceUnit.INCH) - robot.getX(DistanceUnit.INCH) - leadX;
        double dy = target.getY(DistanceUnit.INCH) - robot.getY(DistanceUnit.INCH) - leadY;
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double forwardOffset = POLLEN_SHOOTER_DRIVETRAIN_OFFSET_MM / MM_PER_INCH;
        double leftOffset = TURRET_LEFT_OFFSET_MM / MM_PER_INCH;
        double forward = dx * cos + dy * sin - forwardOffset;
        double left = -dx * sin + dy * cos - leftOffset;
        distanceInches = Math.hypot(forward, left);

        if (!Double.isFinite(distanceInches) || distanceInches < 1e-6) {
            distanceInches = Double.NaN;
            return updateAngle(Double.NaN, true, 0.0);
        }
        double clockwiseDeg = Math.toDegrees(Math.atan2(-left, forward)) + trimDeg;
        double angle = AngleUnit.normalizeDegrees(clockwiseDeg % 360.0);
        return updateAngle(angle, true, 0.0);
    }

    /** Clamps an out-of-range bench angle and returns false. */
    public boolean setAngleDegrees(double angleDeg) {
        distanceInches = Double.NaN;
        return updateAngle(angleDeg, false, 0.0);
    }

    public boolean setAngleDegreesForSyncTest(double angleDeg, double rightServoTrim) {
        distanceInches = Double.NaN;
        if (!Double.isFinite(rightServoTrim)) {
            rightServoTrim = 0.0;
        }
        double trim = Range.clip(rightServoTrim, -MAX_SYNC_TEST_TRIM, MAX_SYNC_TEST_TRIM);
        return updateAngle(angleDeg, false, trim);
    }

    public double getLastServoPositionDifference() {
        return lastRightPosition - lastLeftPosition;
    }

    public double getDistanceInches() {
        return distanceInches;
    }

    private boolean updateAngle(double angleDeg, boolean tracking, double rightServoTrim) {
        requestedDeg = angleDeg;
        limitedDeg = Double.NaN;

        // Snapshot the live calibration for both validation and interpolation.
        double leftAtLeft130 = LEFT_SERVO_AT_LEFT_130;
        double leftAtLeft90 = LEFT_SERVO_AT_LEFT_90;
        double leftCenter = LEFT_SERVO_AT_CENTER;
        double leftAtRight90 = LEFT_SERVO_AT_RIGHT_90;
        double leftAtRight130 = LEFT_SERVO_AT_RIGHT_130;
        double rightAtLeft130 = RIGHT_SERVO_AT_LEFT_130;
        double rightAtLeft90 = RIGHT_SERVO_AT_LEFT_90;
        double rightCenter = RIGHT_SERVO_AT_CENTER;
        double rightAtRight90 = RIGHT_SERVO_AT_RIGHT_90;
        double rightAtRight130 = RIGHT_SERVO_AT_RIGHT_130;
        double requestedHalfTravel = TOTAL_TRAVEL_DEG / 2.0;
        double deadband = tracking ? AIM_DEADBAND_DEG : 0.0;
        if (!Double.isFinite(requestedHalfTravel) || requestedHalfTravel < 0.0
                || !Double.isFinite(deadband) || deadband < 0.0
                || !validCalibration(leftAtLeft130, leftAtLeft90, leftCenter, leftAtRight90, leftAtRight130)
                || !validCalibration(rightAtLeft130, rightAtLeft90, rightCenter, rightAtRight90, rightAtRight130)) {
            allowedHalfTravelDeg = Double.NaN;
            outputStatus = "Check travel limits and calibration";
            return false;
        }
        allowedHalfTravelDeg = Math.min(requestedHalfTravel, MAX_CALIBRATION_DEG);
        if (!Double.isFinite(angleDeg)) {
            outputStatus = "Holding after invalid angle";
            return false;
        }

        double limit = allowedHalfTravelDeg;
        limitedDeg = Range.clip(angleDeg, -limit, limit);
        boolean inRange = Math.abs(angleDeg - limitedDeg) <= 1e-6;
        boolean holdOutOfRange = tracking && !inRange;
        boolean hasPreviousAngle = Double.isFinite(lastCommandedDeg);
        if (holdOutOfRange && !hasPreviousAngle) {
            outputStatus = "Waiting for target in range";
            return false;
        }

        double commandDeg = limitedDeg;
        double previousAngle = Range.clip(lastCommandedDeg, -limit, limit);
        // Compare against the last command so small changes can accumulate.
        boolean smallChange = tracking && hasPreviousAngle && Math.abs(limitedDeg - previousAngle) < deadband;
        if (holdOutOfRange || smallChange) {
            commandDeg = previousAngle;
        }

        double leftPosition = positionForAngle(commandDeg,
                leftAtLeft130, leftAtLeft90, leftCenter, leftAtRight90, leftAtRight130);
        double rightPosition = positionForAngle(commandDeg,
                rightAtLeft130, rightAtLeft90, rightCenter, rightAtRight90, rightAtRight130);
        if (rightServoTrim != 0.0) {
            rightPosition = Range.clip(rightPosition + rightServoTrim, rightAtRight130, rightAtLeft130);
        }
        if (!enabled) {
            outputStatus = "Output disabled";
            return inRange;
        }

        boolean positionChanged = leftPosition != lastLeftPosition || rightPosition != lastRightPosition;
        if (sendOnEnable || positionChanged || servoTimer.milliseconds() >= SERVO_REFRESH_MS) {
            leftTurret.setPosition(leftPosition);
            rightTurret.setPosition(rightPosition);
            lastCommandedDeg = commandDeg;
            lastLeftPosition = leftPosition;
            lastRightPosition = rightPosition;
            sendOnEnable = false;
            servoTimer.reset();
            outputStatus = "Command sent";
        } else {
            outputStatus = "Position steady";
        }
        if (holdOutOfRange) {
            outputStatus = "Holding for out-of-range target";
        } else if (smallChange && commandDeg != limitedDeg) {
            outputStatus = "Holding within deadband";
        }
        return inRange;
    }

    private static boolean validCalibration(double atLeft130, double atLeft90, double center,
                                            double atRight90, double atRight130) {
        return atLeft130 <= 1.0 && atLeft130 > atLeft90 && atLeft90 > center
                && center > atRight90 && atRight90 > atRight130 && atRight130 >= 0.0;
    }

    private static double positionForAngle(double angleDeg, double atLeft130, double atLeft90,
                                           double center, double atRight90, double atRight130) {
        double turn = Math.abs(angleDeg);
        double at90 = angleDeg < 0.0 ? atLeft90 : atRight90;
        double at130 = angleDeg < 0.0 ? atLeft130 : atRight130;
        if (turn <= MID_CALIBRATION_DEG) {
            return interpolate(center, at90, turn / MID_CALIBRATION_DEG);
        }
        double fraction = (turn - MID_CALIBRATION_DEG) / (MAX_CALIBRATION_DEG - MID_CALIBRATION_DEG);
        return interpolate(at90, at130, fraction);
    }

    private static double interpolate(double start, double end, double fraction) {
        return start * (1.0 - fraction) + end * fraction;
    }

    public void addTelemetry(Telemetry telemetry) {
        report(telemetry::addData);
    }

    @Override
    public void report(Report report) {
        Object commandedAngle = Double.isNaN(lastCommandedDeg)
                ? "Waiting for command" : String.format(Locale.US, "%+.1f°", lastCommandedDeg);
        report.data("Turret commanded angle", commandedAngle);
        report.data("Turret status", outputStatus);
        report.data("Turret total / allowed travel (deg)", String.format(Locale.US, "%.1f / %.1f",
                TOTAL_TRAVEL_DEG, 2.0 * allowedHalfTravelDeg));
        report.data("Turret distance (in)", String.format(Locale.US, "%.2f", distanceInches));
        report.data("Turret request / limited (deg)", String.format(Locale.US, "%.1f / %.1f", requestedDeg, limitedDeg));
        report.data("Turret servo L / R", String.format(Locale.US, "%.5f / %.5f", lastLeftPosition, lastRightPosition));
    }
}
