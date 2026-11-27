/*
 * PinpointTeleOp.java - drive the robot, and measure goBILDA Pinpoint parameters.
 *
 * Adapted from PosConTeleOp for the goBILDA Pinpoint Odometry Computer.
 */

package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

/*
 * Drive the robot on four mecanum wheels, and walk through the four steps that
 * measure and verify the goBILDA Pinpoint configuration.
 *
 * SETUP
 *
 * Configure the four drive motors as "lf", "rf", "lb" and "rb", and the Pinpoint
 * as an I2C device of type "goBILDA® Pinpoint Odometry Computer" named "pp".
 * Use the same hardware name as the competition programs.
 *
 * Plug the forward pod into X and the strafe pod into Y.
 *
 * CONTROLS
 *
 * Left stick          forward, back, strafe
 * Right stick X       turn
 * Bumpers             previous / next setup step
 * A, B, X, Y, dpad    whatever the step on screen says
 *
 * WHAT THE FOUR STEPS DO
 *
 * 1. DIRECTION   which way each pod counts
 * 2. SCALE       how far one tick is (presets or tape-measure calibration)
 * 3. OFFSETS     where each pod sits relative to the tracking centre (spin test)
 * 4. VERIFY      drive a closed loop, check closure error, output code snippet
 */
@TeleOp(name = "Pinpoint Auto Tuner", group = "Pinpoint")
public class PinpointTeleOp extends LinearOpMode {

    private enum Step { DIRECTION, SCALE, OFFSETS, VERIFY }

    private enum ScaleMode { MENU, MEASURE_X, MEASURE_Y }

    /** A closure error at or under this is as good as this method gets. */
    private static final double GOOD_ERROR_IN = 0.4;
    private static final double GOOD_ERROR_DEG = 2.0;

    /** Where the robot is declared to be at every zero: the middle of the field. */
    private static final double START_X_IN = 72.0, START_Y_IN = 72.0, START_H_DEG = 0.0;

    /** Below this a step-2 measurement is too short to divide by. */
    private static final int MIN_MEASURE_TICKS = 100;

    /** Turns step 3 wants before it will believe the answer. More is better. */
    private static final double MIN_SPIN_TURNS = 2.0;

    /** Sanity bound on an offset in inches. */
    private static final double MAX_OFFSET_IN = 40.0;

    /** Default resolution: goBILDA 4-Bar Pod (19.89436789 ticks/mm -> ~505.3 ticks/in). */
    private static final double TICKS_PER_IN_4_BAR = 19.89436789 * 25.4;
    /** goBILDA Swingarm Pod (13.26291192 ticks/mm -> ~336.9 ticks/in). */
    private static final double TICKS_PER_IN_SWINGARM = 13.26291192 * 25.4;

    /** Footprint drawn on the field view, in inches. */
    private static final double ROBOT_LENGTH_IN = 18.0;
    private static final double ROBOT_WIDTH_IN = 18.0;

    /** Trail: one point per this many inches of travel, this many kept. */
    private static final double TRAIL_STEP_IN = 2.0;
    private static final int TRAIL_MAX = 60;

    // Field-view colours.
    private static final String C_BODY = "#e8edf2";
    private static final String C_NOSE = "#ff4d4d";
    private static final String C_DOT = "#ffd166";
    private static final String C_TRAIL = "#4ea1ff";
    private static final String C_STALE = "#6b7480";
    private static final String C_NONE = "transparent";

    /** Trail ring buffer, in field inches. */
    private final double[] trailX = new double[TRAIL_MAX];
    private final double[] trailY = new double[TRAIL_MAX];
    private int trailCount = 0, trailHead = 0;

    private Step step = Step.DIRECTION;
    private ScaleMode scaleMode = ScaleMode.MENU;

    // Pinpoint configuration state mirrored in Java (since Pinpoint does not
    // provide getters for encoder direction or resolution).
    private GoBildaPinpointDriver.EncoderDirection dirX = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private GoBildaPinpointDriver.EncoderDirection dirY = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private GoBildaPinpointDriver.GoBildaOdometryPods activePreset = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    private double resTicksPerIn = TICKS_PER_IN_4_BAR;
    private double measuredResX = 0.0;
    private double offXIn = 0.0, offYIn = 0.0;

    // Steps 1, 3 and 4 all work from "how far since I marked this spot".
    private int markX, markY;
    private double markH;

    // Step 2, measuring: the distance the robot was actually moved.
    private double measureIn = 24.0;
    private String scaleNote = "";

    private String dirNote = "";
    private String offsetNote = "";
    private String saveNote = "";

    /** Set by a direction flip; the zero it wants happens the next loop. */
    private boolean pendingZero;

    // Previous button states, for rising-edge detection.
    private boolean pA, pB, pX, pY, pLB, pRB, pUp, pDown, pLeft, pRight;

    @Override
    public void runOpMode() {
        DcMotor lf = hardwareMap.get(DcMotor.class, "lf");
        DcMotor rf = hardwareMap.get(DcMotor.class, "rf");
        DcMotor lb = hardwareMap.get(DcMotor.class, "lb");
        DcMotor rb = hardwareMap.get(DcMotor.class, "rb");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GoBildaPinpointDriver odo;
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pp");

        GamepadManager pad = PanelsGamepad.INSTANCE.getFirstManager();

        // One telemetry that writes to both the Driver Station and Panels.
        Telemetry tel = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        FieldManager field;
        try {
            field = PanelsField.INSTANCE.getField();
        } catch (Throwable t) {
            field = null;
        }

        // Seed initial configuration onto the Pinpoint and read any existing offsets.
        odo.setEncoderDirections(dirX, dirY);
        odo.setEncoderResolution(activePreset);
        odo.recalibrateIMU();
        sleep(300); // Wait for IMU calibration to settle

        odo.update();
        offXIn = odo.getXOffset(DistanceUnit.INCH);
        offYIn = odo.getYOffset(DistanceUnit.INCH);
        if (Double.isNaN(offXIn)) offXIn = 0.0;
        if (Double.isNaN(offYIn)) offYIn = 0.0;

        tel.addLine("Left stick drives and strafes. Right stick turns.");
        tel.addLine("Bumpers move through the four setup steps.");
        tel.addLine("");
        tel.addLine("Forward pod in X, strafe pod in Y.");
        tel.addData("Pinpoint Status", odo.getDeviceStatus());
        tel.update();

        waitForStart();

        // Start from a known pose every run.
        odo.setPosition(startPose());
        odo.update();
        mark(odo);

        while (opModeIsActive()) {
            Gamepad g = pad.asCombinedFTCGamepad(gamepad1);
            odo.update();

            double y = -g.left_stick_y;
            double x = g.left_stick_x;
            double rx = g.right_stick_x;

            lf.setPower(y + x + rx);
            rf.setPower(y - x - rx);
            lb.setPower(y - x + rx);
            rb.setPower(y + x - rx);

            // Rising edges, so one press is one action however long it is held.
            boolean a = g.a && !pA, b = g.b && !pB, xb = g.x && !pX, yb = g.y && !pY;
            boolean lbmp = g.left_bumper && !pLB, rbmp = g.right_bumper && !pRB;
            boolean up = g.dpad_up && !pUp, down = g.dpad_down && !pDown;
            boolean left = g.dpad_left && !pLeft, right = g.dpad_right && !pRight;
            pA = g.a; pB = g.b; pX = g.x; pY = g.y;
            pLB = g.left_bumper; pRB = g.right_bumper;
            pUp = g.dpad_up; pDown = g.dpad_down;
            pLeft = g.dpad_left; pRight = g.dpad_right;

            if (scaleMode == ScaleMode.MENU) {
                if (rbmp) {
                    step = nextStep(step);
                }
                if (lbmp) {
                    step = prevStep(step);
                }
            }

            switch (step) {
                case DIRECTION:
                    stepDirection(odo, tel, a, xb, yb);
                    break;
                case SCALE:
                    stepScale(odo, tel, a, b, yb, up, down, left, right);
                    break;
                case OFFSETS:
                    stepOffsets(odo, tel, a, b);
                    break;
                case VERIFY:
                    stepVerify(odo, tel, a, b);
                    break;
            }

            board(odo, tel);
            tel.update();

            // Field view rendering
            Pose2D shown = odo.getPosition();
            double fx = shown.getX(DistanceUnit.INCH);
            double fy = shown.getY(DistanceUnit.INCH);
            pushTrail(fx, fy);
            if (field != null) {
                drawField(field, fx, fy, shown.getHeading(AngleUnit.RADIANS),
                        odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY);
            }
        }
    }

    // ====================================================================
    //  Step 1 -- which way each pod counts
    // ====================================================================

    // Pinpoint reports raw encoder counts. signed() applies the configured direction.
    private void stepDirection(GoBildaPinpointDriver odo, Telemetry tel,
                               boolean zero, boolean flipX, boolean flipY) {
        if (flipX) {
            dirX = flip(dirX);
        }
        if (flipY) {
            dirY = flip(dirY);
        }
        if (flipX || flipY) {
            applyDirectionsReliably(odo, dirX, dirY);
            pendingZero = true;
            dirNote = String.format(Locale.US, "Flipped ENC %s - Zero reset. Push the robot again to retest.",
                    flipX && flipY ? "X and Y" : flipX ? "X" : "Y");
        } else if (pendingZero) {
            pendingZero = false;
            zeroHere(odo);
        }
        if (zero) {
            zeroHere(odo);
            dirNote = "";
        }

        int rawDx = odo.getEncoderX() - markX;
        int rawDy = odo.getEncoderY() - markY;
        Pose2D p = odo.getPosition();

        tel.addLine("STEP 1 of 4   POD DIRECTION");
        tel.addLine("Push forward about 1 ft. X encoder counts and X distance should increase (+).");
        tel.addLine("Push left about 1 ft. Y encoder counts and Y distance should increase (+).");
        tel.addLine("If either reading decreases, press X or Y to reverse that axis.");
        tel.addLine("");
        tel.addData("ENC X", "counts %+d   %s   (raw pod %+d)   [X flips]",
                signed(rawDx, dirX), dirX, rawDx);
        tel.addData("ENC Y", "counts %+d   %s   (raw pod %+d)   [Y flips]",
                signed(rawDy, dirY), dirY, rawDy);
        tel.addData("moved", "x %+.1f in   y %+.1f in   h %+.1f deg",
                p.getX(DistanceUnit.INCH) - START_X_IN,
                p.getY(DistanceUnit.INCH) - START_Y_IN,
                p.getHeading(AngleUnit.DEGREES) - START_H_DEG);
        tel.addLine("Use the counts and distance above to verify direction. Raw pod values stay the same when flipped.");
        if (!dirNote.isEmpty()) {
            tel.addLine(dirNote);
        }
        tel.addLine("");
        tel.addLine("A = Zero here       RB = Next step (Scale)");
    }

    private static int signed(int ticks, GoBildaPinpointDriver.EncoderDirection dir) {
        return dir == GoBildaPinpointDriver.EncoderDirection.REVERSED ? -ticks : ticks;
    }

    /** Separate writes to DEVICE_CONTROL so the I2C queue retains both direction commands. */
    private void applyDirectionsReliably(GoBildaPinpointDriver odo,
                                         GoBildaPinpointDriver.EncoderDirection x,
                                         GoBildaPinpointDriver.EncoderDirection y) {
        odo.setEncoderDirections(x, y);
        sleep(25);
        odo.setEncoderDirections(x, y);
        sleep(25);
    }

    private static boolean isSane(double offsetIn) {
        return !Double.isNaN(offsetIn) && !Double.isInfinite(offsetIn)
                && Math.abs(offsetIn) <= MAX_OFFSET_IN;
    }

    // ====================================================================
    //  Step 2 -- how far one tick is
    // ====================================================================

    // Pinpoint shares a single MM_PER_TICK register for both pods.
    private void stepScale(GoBildaPinpointDriver odo, Telemetry tel, boolean markHere,
                           boolean accept, boolean cancel,
                           boolean up, boolean down, boolean left, boolean right) {
        if (scaleMode == ScaleMode.MENU) {
            if (up) {
                applyPreset(odo, GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD, TICKS_PER_IN_4_BAR);
            }
            if (down) {
                applyPreset(odo, GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD, TICKS_PER_IN_SWINGARM);
            }
            if (right) {
                scaleMode = ScaleMode.MEASURE_X;
                scaleNote = "";
                mark(odo);
            }

            tel.addLine("STEP 2 of 4   POD SCALE");
            tel.addData("current scale", "%.2f ticks/in (%.4f ticks/mm)%s",
                    resTicksPerIn, resTicksPerIn / 25.4,
                    activePreset != null ? " [" + activePreset.name() + "]" : " [CUSTOM]");
            tel.addLine("");
            tel.addLine("DPAD UP     goBILDA 4-Bar pod    (32 mm wheel)");
            tel.addLine("DPAD DOWN   goBILDA Swingarm pod (48 mm wheel)");
            tel.addLine("DPAD RIGHT  measure it instead");
            if (!scaleNote.isEmpty()) {
                tel.addLine("");
                tel.addLine(scaleNote);
            }
            tel.addLine("");
            tel.addLine("LB = back to direction    RB = on to the offsets");
            return;
        }

        boolean measuringX = scaleMode == ScaleMode.MEASURE_X;

        if (cancel) {
            scaleMode = ScaleMode.MENU;
            scaleNote = "Measurement cancelled - kept previous resolution.";
            return;
        }
        if (markHere) {
            mark(odo);
            scaleNote = "";
        }
        if (up) {
            measureIn = clampDistance(measureIn + 1.0);
        }
        if (down) {
            measureIn = clampDistance(measureIn - 1.0);
        }
        if (right) {
            measureIn = clampDistance(measureIn + 0.1);
        }
        if (left) {
            measureIn = clampDistance(measureIn - 0.1);
        }

        int ticks = measuringX ? odo.getEncoderX() - markX : odo.getEncoderY() - markY;
        double ticksPerIn = Math.abs(ticks) / measureIn;

        if (accept) {
            applyMeasured(odo, measuringX, Math.abs(ticks), ticksPerIn);
            return;
        }

        tel.addLine(measuringX ? "STEP 2 of 4   MEASURING ENC X"
                               : "STEP 2 of 4   MEASURING ENC Y (Optional)");
        tel.addLine(measuringX ? "Drive the robot straight FORWARD."
                               : "Drive the robot straight LEFT (or press Y to keep X scale).");
        tel.addLine("Longer is better - the error in your tape reading is");
        tel.addLine("divided by the distance you drove.");
        tel.addLine("");
        tel.addData("1. A", "mark the start (do this before moving)");
        tel.addData("2. dpad", "distance actually moved = %.1f in", measureIn);
        tel.addLine("          up/down 1 in, left/right 0.1 in");
        tel.addData("3. B", "accept  ->  %.2f ticks/in", ticksPerIn);
        tel.addLine("");
        tel.addData("ticks since mark", "%+d", ticks);
        if (!scaleNote.isEmpty()) {
            tel.addLine(scaleNote);
        }
        tel.addLine(measuringX ? "Y = cancel" : "Y = finish using ENC X scale only");
    }

    private void applyPreset(GoBildaPinpointDriver odo,
                             GoBildaPinpointDriver.GoBildaOdometryPods pods,
                             double ticksPerIn) {
        odo.setEncoderResolution(pods);
        activePreset = pods;
        resTicksPerIn = ticksPerIn;
        scaleNote = "Set pods to " + pods.name() + ".";
    }

    private void applyMeasured(GoBildaPinpointDriver odo, boolean isX, int ticks, double ticksPerIn) {
        if (ticks < MIN_MEASURE_TICKS) {
            scaleNote = "Only " + ticks + " ticks - move the robot further and try again.";
            return;
        }
        activePreset = null;
        if (isX) {
            measuredResX = ticksPerIn;
            resTicksPerIn = ticksPerIn;
            odo.setEncoderResolution(resTicksPerIn, DistanceUnit.INCH);
            scaleMode = ScaleMode.MEASURE_Y;
            scaleNote = String.format(Locale.US,
                    "ENC X set to %.2f ticks/in. Measure Y to average, or press Y to finish.",
                    resTicksPerIn);
            mark(odo);
        } else {
            resTicksPerIn = (measuredResX + ticksPerIn) / 2.0;
            odo.setEncoderResolution(resTicksPerIn, DistanceUnit.INCH);
            scaleMode = ScaleMode.MENU;
            scaleNote = String.format(Locale.US,
                    "Averaged X (%.1f) & Y (%.1f) -> %.2f ticks/in.",
                    measuredResX, ticksPerIn, resTicksPerIn);
        }
    }

    // ====================================================================
    //  Step 3 -- where each pod sits
    // ====================================================================

    // Pure rotation model matching GoBildaPinpointDriver conventions:
    //   xOffset (forward pod sideways offset, left +) = -dx_pod / dtheta
    //   yOffset (strafe pod forward offset, ahead +)  = +dy_pod / dtheta
    private void stepOffsets(GoBildaPinpointDriver odo, Telemetry tel,
                             boolean markHere, boolean accept) {
        if (markHere) {
            mark(odo);
            offsetNote = "";
        }

        double turnRad = odo.getHeading(UnnormalizedAngleUnit.RADIANS) - markH;
        double turns = Math.abs(turnRad) / (2 * Math.PI);

        double dxPodIn = signed(odo.getEncoderX() - markX, dirX) / resTicksPerIn;
        double dyPodIn = signed(odo.getEncoderY() - markY, dirY) / resTicksPerIn;

        boolean enough = turns >= MIN_SPIN_TURNS;
        double newOffX = enough ? -dxPodIn / turnRad : 0.0;
        double newOffY = enough ? dyPodIn / turnRad : 0.0;

        if (accept) {
            applySpin(odo, enough, turns, newOffX, newOffY);
            return;
        }

        tel.addLine("STEP 3 of 4   POD OFFSETS");
        tel.addLine("Spin the robot in place, over the spot you want tracked.");
        tel.addLine("At least " + (int) MIN_SPIN_TURNS + " full turns - more is better,");
        tel.addLine("and either direction works.");
        tel.addLine("");
        tel.addData("1. A", "mark the start (do this before spinning)");
        tel.addData("2.   ", "spin: turns so far %.2f %s", turns,
                enough ? "- enough" : "- keep going");
        tel.addData("3. B", "accept and write to Pinpoint");
        tel.addLine("");
        if (enough) {
            tel.addData("ENC X offset", "%+.2f in (%+.1f mm)  forward pod is %s of centre",
                    newOffX, newOffX * 25.4, newOffX >= 0 ? "LEFT" : "RIGHT");
            tel.addData("ENC Y offset", "%+.2f in (%+.1f mm)  strafe pod is %s of centre",
                    newOffY, newOffY * 25.4, newOffY >= 0 ? "AHEAD" : "BEHIND");
        } else {
            tel.addLine("(the offsets appear once you have turned far enough)");
        }
        tel.addData("on the board", "X %+.2f in (%+.1f mm)   Y %+.2f in (%+.1f mm)",
                offXIn, offXIn * 25.4, offYIn, offYIn * 25.4);
        if (!offsetNote.isEmpty()) {
            tel.addLine("");
            tel.addLine(offsetNote);
        }
        tel.addLine("");
        tel.addLine("LB = back to the scale    RB = on to verify");
    }

    private void applySpin(GoBildaPinpointDriver odo, boolean enough, double turns,
                           double newOffX, double newOffY) {
        if (!enough) {
            offsetNote = String.format(Locale.US,
                    "Only %.2f turns - spin at least %.0f and try again.",
                    turns, MIN_SPIN_TURNS);
            return;
        }
        if (!isSane(newOffX) || !isSane(newOffY)) {
            offsetNote = "That works out past " + (int) MAX_OFFSET_IN
                    + " in, which no robot is. Check step 2 and spin again.";
            return;
        }
        offXIn = newOffX;
        offYIn = newOffY;
        odo.setOffsets(offXIn, offYIn, DistanceUnit.INCH);
        offsetNote = String.format(Locale.US,
                "Applied from %.2f turns. Zero and spin again to verify pose stays put.",
                turns);
    }

    // ====================================================================
    //  Step 4 -- prove it, then export code snippet
    // ====================================================================

    private void stepVerify(GoBildaPinpointDriver odo, Telemetry tel, boolean zero, boolean dumpLog) {
        if (zero) {
            zeroHere(odo);
            saveNote = "";
        }
        if (dumpLog) {
            String snippet = buildCodeSnippet();
            RobotLog.ii("PinpointTuner", "\n" + snippet);
            saveNote = "Wrote config snippet to RobotLog (Logcat tag: PinpointTuner).";
        }

        Pose2D p = odo.getPosition();
        double px = p.getX(DistanceUnit.INCH);
        double py = p.getY(DistanceUnit.INCH);
        double ph = p.getHeading(AngleUnit.DEGREES);
        double err = Math.hypot(px - START_X_IN, py - START_Y_IN);
        double errH = Math.abs(ph - START_H_DEG);

        tel.addLine("STEP 4 of 4   VERIFY AND EXPORT");
        tel.addData("1. A", "zero pose at field centre (%.0f, %.0f)", START_X_IN, START_Y_IN);
        tel.addLine("2.    drive a loop and return to the exact start spot & heading");
        tel.addData("3. B", "dump Java config code to Logcat");
        tel.addLine("");
        tel.addData("pose", "x %.1f in   y %.1f in   h %+.1f deg", px, py, ph);
        tel.addData("closure", "%.2f in, %.1f deg   %s", err, errH,
                err <= GOOD_ERROR_IN && errH <= GOOD_ERROR_DEG
                        ? "- good" : "- go back a step if this is large");
        tel.addLine("");
        tel.addLine("--- COPY INTO YOUR OPMODE / CONSTANTS ---");
        tel.addLine(String.format(Locale.US,
                "odo.setEncoderDirections(EncoderDirection.%s, EncoderDirection.%s);",
                dirX, dirY));
        if (activePreset != null) {
            tel.addLine(String.format(Locale.US,
                    "odo.setEncoderResolution(GoBildaOdometryPods.%s);", activePreset));
        } else {
            tel.addLine(String.format(Locale.US,
                    "odo.setEncoderResolution(%.4f, DistanceUnit.INCH);", resTicksPerIn));
        }
        tel.addLine(String.format(Locale.US,
                "odo.setOffsets(%.3f, %.3f, DistanceUnit.INCH); // (%.1f, %.1f mm)",
                offXIn, offYIn, offXIn * 25.4, offYIn * 25.4));
        if (!saveNote.isEmpty()) {
            tel.addLine("");
            tel.addLine(saveNote);
        }
        tel.addLine("");
        tel.addLine("LB = back to the offsets");
    }

    private String buildCodeSnippet() {
        StringBuilder sb = new StringBuilder();
        sb.append("// --- goBILDA Pinpoint Tuned Configuration ---\n");
        sb.append(String.format(Locale.US,
                "odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.%s, GoBildaPinpointDriver.EncoderDirection.%s);\n",
                dirX, dirY));
        if (activePreset != null) {
            sb.append(String.format(Locale.US,
                    "odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.%s);\n",
                    activePreset));
        } else {
            sb.append(String.format(Locale.US,
                    "odo.setEncoderResolution(%.4f, DistanceUnit.INCH);\n", resTicksPerIn));
        }
        sb.append(String.format(Locale.US,
                "odo.setOffsets(%.4f, %.4f, DistanceUnit.INCH); // %.2f mm, %.2f mm\n",
                offXIn, offYIn, offXIn * 25.4, offYIn * 25.4));
        return sb.toString();
    }

    // ====================================================================
    //  Shared
    // ====================================================================

    /** What the Pinpoint itself reports, on every screen. */
    private void board(GoBildaPinpointDriver odo, Telemetry tel) {
        GoBildaPinpointDriver.DeviceStatus st = odo.getDeviceStatus();
        tel.addLine("");
        tel.addData("pinpoint", "%s (LED %s) %.0f Hz",
                st, getLedColor(st), odo.getFrequency());
        if (st != GoBildaPinpointDriver.DeviceStatus.READY) {
            tel.addLine(getStatusDescription(st));
        }
    }

    private static String getLedColor(GoBildaPinpointDriver.DeviceStatus st) {
        switch (st) {
            case READY: return "green";
            case NOT_READY:
            case CALIBRATING:
            case FAULT_IMU_RUNAWAY: return "red";
            case FAULT_X_POD_NOT_DETECTED: return "blue";
            case FAULT_Y_POD_NOT_DETECTED: return "orange";
            case FAULT_NO_PODS_DETECTED: return "purple";
            case FAULT_BAD_READ: return "unaffected";
            default: return "unknown";
        }
    }

    private static String getStatusDescription(GoBildaPinpointDriver.DeviceStatus st) {
        switch (st) {
            case NOT_READY: return "Powering up - hold still.";
            case CALIBRATING: return "Calibrating gyro - hold the robot still.";
            case FAULT_X_POD_NOT_DETECTED: return "Forward (X) pod not detected - check cable.";
            case FAULT_Y_POD_NOT_DETECTED: return "Strafe (Y) pod not detected - check cable.";
            case FAULT_NO_PODS_DETECTED: return "Neither X nor Y pod detected.";
            case FAULT_IMU_RUNAWAY: return "IMU runaway detected - recalibrate or service.";
            case FAULT_BAD_READ: return "Bad I2C read detected - holding last good value.";
            default: return "";
        }
    }

    private void mark(GoBildaPinpointDriver odo) {
        markX = odo.getEncoderX();
        markY = odo.getEncoderY();
        markH = odo.getHeading(UnnormalizedAngleUnit.RADIANS);
    }

    private void zeroHere(GoBildaPinpointDriver odo) {
        odo.setPosition(startPose());
        odo.update();
        mark(odo);
        clearTrail();
    }

    private static Pose2D startPose() {
        return new Pose2D(DistanceUnit.INCH, START_X_IN, START_Y_IN,
                AngleUnit.DEGREES, START_H_DEG);
    }

    private static double clampDistance(double in) {
        return Math.max(1.0, Math.min(240.0, Math.round(in * 10.0) / 10.0));
    }

    private static GoBildaPinpointDriver.EncoderDirection flip(GoBildaPinpointDriver.EncoderDirection d) {
        return d == GoBildaPinpointDriver.EncoderDirection.FORWARD
                ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                : GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    private static Step nextStep(Step s) {
        Step[] all = Step.values();
        return all[Math.min(s.ordinal() + 1, all.length - 1)];
    }

    private static Step prevStep(Step s) {
        return Step.values()[Math.max(s.ordinal() - 1, 0)];
    }

    // ====================================================================
    //  The field view
    // ====================================================================

    private void clearTrail() {
        trailCount = 0;
        trailHead = 0;
    }

    private void pushTrail(double x, double y) {
        if (trailCount > 0) {
            int last = (trailHead + trailCount - 1) % TRAIL_MAX;
            double dx = x - trailX[last], dy = y - trailY[last];
            if (dx * dx + dy * dy < TRAIL_STEP_IN * TRAIL_STEP_IN) {
                return;
            }
        }
        int slot = (trailHead + trailCount) % TRAIL_MAX;
        trailX[slot] = x;
        trailY[slot] = y;
        if (trailCount < TRAIL_MAX) {
            trailCount++;
        } else {
            trailHead = (trailHead + 1) % TRAIL_MAX;
        }
    }

    private void drawField(FieldManager field, double x, double y,
                           double headingRad, boolean live) {
        if (trailCount > 1) {
            field.setStyle(C_NONE, C_TRAIL, 0.4);
            for (int i = 0; i < trailCount - 1; i++) {
                int a = (trailHead + i) % TRAIL_MAX;
                int b = (trailHead + i + 1) % TRAIL_MAX;
                field.moveCursor(trailX[a], trailY[a]);
                field.line(trailX[b], trailY[b]);
            }
            int last = (trailHead + trailCount - 1) % TRAIL_MAX;
            field.moveCursor(trailX[last], trailY[last]);
            field.line(x, y);
        }

        double c = Math.cos(headingRad), s = Math.sin(headingRad);
        double hl = ROBOT_LENGTH_IN / 2, hw = ROBOT_WIDTH_IN / 2;
        double[][] local = {{hl, hw}, {-hl, hw}, {-hl, -hw}, {hl, -hw}};
        double[] cx = new double[4], cy = new double[4];
        for (int i = 0; i < 4; i++) {
            cx[i] = x + local[i][0] * c - local[i][1] * s;
            cy[i] = y + local[i][0] * s + local[i][1] * c;
        }

        field.setStyle(C_NONE, live ? C_BODY : C_STALE, 0.7);
        for (int i = 0; i < 3; i++) {
            field.moveCursor(cx[i], cy[i]);
            field.line(cx[i + 1], cy[i + 1]);
        }

        field.setStyle(C_NONE, live ? C_NOSE : C_STALE, 1.4);
        field.moveCursor(cx[3], cy[3]);
        field.line(cx[0], cy[0]);
        field.moveCursor(x, y);
        field.line(x + hl * 1.6 * c, y + hl * 1.6 * s);

        field.setStyle(live ? C_DOT : C_STALE, C_NONE, 0.0);
        field.moveCursor(x, y);
        field.circle(1.3);

        field.update();
    }
}