package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.subsystems.autoTurretAim.TOTAL_TRAVEL_DEG;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.autoTurretAim;

/** Bench-check both servo calibrations before verifying field tracking with Pinpoint. */
@Configurable
@TeleOp(name = "Auto Turret Aim Test", group = "Test")
public class AutoTurretAimTest extends LinearOpMode {
    public static boolean USE_PINPOINT = false;
    public static double ROBOT_X_IN = 70.75;
    public static double ROBOT_Y_IN = 9;
    public static double ROBOT_H_DEG = 90.0;
    public static double TARGET_X_IN = 58.0;
    public static double TARGET_Y_IN = 55.0;
    public static double TRIM_DEG = 0.0;
    public static boolean DIRECT_ANGLE_MODE = false;
    public static double DIRECT_ANGLE_DEG = 0.0;
    public static double SYNC_RIGHT_TRIM = 0.008;

    private enum Preset { NONE, LEFT, CENTER, RIGHT }

    @Override
    public void runOpMode() {
        autoTurretAim aim = new autoTurretAim(hardwareMap);
        JoinedTelemetry display = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        display.addLine("Set test values in Panels. Press A after START to enable or disable turret output.");
        display.addLine("D-pad left/up/right selects left/center/right. Press X to exit the preset.");
        display.update();
        waitForStart();
        if (isStopRequested()) return;

        Drivetrain drivetrain = null;
        if (USE_PINPOINT) {
            drivetrain = new Drivetrain();
            drivetrain.init(hardwareMap);
            drivetrain.setPosition(configuredPose());
        }

        boolean enabled = false;
        Preset preset = Preset.NONE;
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) enabled = !enabled;
            boolean leftPressed = gamepad1.dpadLeftWasPressed();
            boolean rightPressed = gamepad1.dpadRightWasPressed();
            if (gamepad1.xWasPressed()) {
                preset = Preset.NONE;
            } else if (gamepad1.dpadUpWasPressed()) {
                preset = Preset.CENTER;
            } else if (leftPressed && !rightPressed) {
                preset = Preset.LEFT;
            } else if (rightPressed && !leftPressed) {
                preset = Preset.RIGHT;
            }

            Pose2D current = configuredPose();
            boolean poseReady = true;

            if (drivetrain != null) {
                drivetrain.update();
                current = drivetrain.getPosition();
                GoBildaPinpointDriver.DeviceStatus status = drivetrain.getLocalizationStatus();
                poseReady = status == GoBildaPinpointDriver.DeviceStatus.READY;
                display.addData("Pinpoint", status);
            }

            boolean directAngle = preset != Preset.NONE || DIRECT_ANGLE_MODE;
            aim.setEnabled(enabled && (directAngle || poseReady));

            double halfTravel = TOTAL_TRAVEL_DEG / 2.0;
            String mode;
            if (directAngle) {
                double angleDeg;
                switch (preset) {
                    case LEFT:
                        mode = "Left preset";
                        angleDeg = -halfTravel;
                        break;
                    case CENTER:
                        mode = "Center preset";
                        angleDeg = 0.0;
                        break;
                    case RIGHT:
                        mode = "Right preset";
                        angleDeg = halfTravel;
                        break;
                    default:
                        mode = "Direct angle";
                        angleDeg = DIRECT_ANGLE_DEG;
                }
                if (SYNC_RIGHT_TRIM == 0.0) {
                    aim.setAngleDegrees(angleDeg);
                } else {
                    aim.setAngleDegreesForSyncTest(angleDeg, SYNC_RIGHT_TRIM);
                }
            } else {
                mode = "Field target";
                Pose2D target = new Pose2D(DistanceUnit.INCH, TARGET_X_IN, TARGET_Y_IN, AngleUnit.DEGREES, 0.0);
                aim.aimAt(current, target, TRIM_DEG);
            }

            display.addData("Turret mode", mode);
            display.addData("Output switch (A)", enabled);
            display.addData("Left / center / right (deg)", "%.1f / 0 / %.1f", -halfTravel, halfTravel);
            display.addData("Robot X / Y / H", "%.1f / %.1f / %.1f",
                    current.getX(DistanceUnit.INCH), current.getY(DistanceUnit.INCH),
                    current.getHeading(AngleUnit.DEGREES));
            display.addData("Target X / Y (in)", "%.1f / %.1f", TARGET_X_IN, TARGET_Y_IN);
            aim.addTelemetry(display);
            if (directAngle) {
                display.addData("右舵机同步微调（请求）", "%+.5f", SYNC_RIGHT_TRIM);
            } else {
                display.addData("右舵机同步微调", "直角度模式可调");
            }
            double positionDifference = aim.getLastServoPositionDifference();
            if (Double.isFinite(positionDifference)) {
                display.addData("舵机位置指令差（右减左）", "%+.5f", positionDifference);
            } else {
                display.addData("舵机位置指令差（右减左）", "等待下发");
            }
            display.update();
            sleep(20);
        }
    }

    private static Pose2D configuredPose() {
        return new Pose2D(DistanceUnit.INCH, ROBOT_X_IN, ROBOT_Y_IN, AngleUnit.DEGREES, ROBOT_H_DEG);
    }
}
