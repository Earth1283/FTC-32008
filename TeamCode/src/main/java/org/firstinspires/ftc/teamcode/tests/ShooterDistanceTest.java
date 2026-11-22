package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import java.util.Locale;

/** Record each sample with gamepad 2 X after the flywheel speed stabilizes. */
@Configurable
@TeleOp(name = "Shooter Distance Test", group = "Test")
public final class ShooterDistanceTest extends LinearOpMode {
    public static double START_X_IN = 71.0;
    public static double START_Y_IN = 9.0;
    public static double START_H_DEG = 90.0;

    public static double LOW_TARGET_X_IN = 58.0;
    public static double LOW_TARGET_Y_IN = 55.0;
    public static double HIGH_TARGET_X_IN = 58.0;
    public static double HIGH_TARGET_Y_IN = 85.0;
    public static double MID_Y_IN = 70.75;
    public static double TARGET_SWITCH_BUFFER_IN = 2.0;

    public static double FLYWHEEL_START_TICKS_PER_SECOND = 1400.0;
    public static double PANEL_START_POSITION = 0.50;
    private static final double MAX_TEST_TICKS_PER_SECOND = 2800.0;
    private static final double FLYWHEEL_STEP_TICKS_PER_SECOND = 10.0;
    private static final double AIM_STEP_DEG = 2.0;
    private static final double PANEL_STEP = 0.01;
    private static final double TRIGGER_THRESHOLD = 0.05;
    private static final double TELEMETRY_INTERVAL_MS = 100.0;

    private int sampleCount;
    private String lastSample = "还没有记录";
    private String recordStatus = "转速稳定后按手柄 2 的 X";

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain();
        Intake intake = new Intake();
        Shooter shooter = new Shooter();
        drivetrain.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        autoTurretAim turret = new autoTurretAim(hardwareMap);
        JoinedTelemetry display = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        display.addLine("在 Panels 的 ShooterDistanceTest 中填写初始坐标和测试靶点。");
        display.addLine("手柄 1 右扳机收球，左扳机吐球，左肩开关飞轮，右肩供球。");
        display.addLine("手柄 2 上/下调速，左/右微调炮台，Y/A 调浮板，X 记录测点。");
        display.update();
        intake.shovelUp();
        waitForStart();
        if (isStopRequested()) return;

        double startX = START_X_IN;
        double startY = START_Y_IN;
        double startHeading = START_H_DEG;
        if (!Double.isFinite(startX) || !Double.isFinite(startY) || !Double.isFinite(startHeading)) {
            display.addLine("初始摆位无效，请检查 START_X_IN、START_Y_IN、START_H_DEG。");
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
                drivetrain.update();
                drivetrain.drive(gamepad1, 1.0);
                Pose2D current = drivetrain.getPosition();
                double x = current.getX(DistanceUnit.INCH);
                double y = current.getY(DistanceUnit.INCH);
                double heading = current.getHeading(AngleUnit.DEGREES);
                DeviceStatus status = drivetrain.getLocalizationStatus();
                boolean poseReady = status == DeviceStatus.READY
                        && Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(heading);

                // Hysteresis prevents target changes near midfield.
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

                turret.setEnabled(poseReady);
                double distanceInches = Double.NaN;
                if (poseReady) {
                    turret.aimAt(current, target, aimTrim);
                    distanceInches = turret.getDistanceInches();
                }

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

                if (gamepad1.right_trigger > TRIGGER_THRESHOLD) intake.intakeIn();
                else if (gamepad1.left_trigger > TRIGGER_THRESHOLD) intake.intakeOut();
                else if (gamepad1.right_bumper) intake.intakeFeed();
                else intake.intakeStop();
                if (gamepad1.yWasPressed()) intake.shovelUp();
                if (gamepad1.aWasPressed()) intake.shovelDown();

                if (gamepad2.xWasPressed()) {
                    recordSample(target, distanceInches, runFlywheel, flywheelTarget, measuredVelocity, appliedPanel);
                }
                if (!showTelemetry) continue;
                display.addData("Pinpoint", status);
                display.addData("机器人 X / Y / H", "%.1f / %.1f / %.1f", x, y, heading);
                display.addData("当前靶点 X / Y（英寸）", "%.1f / %.1f", targetX, targetY);
                display.addData("炮台补偿（度）", "%+.1f", aimTrim);
                display.addData("飞轮", runFlywheel ? "开启" : "停止");
                display.addData("转速目标 / 实测（ticks/s）", "%.0f / %.0f", flywheelTarget, measuredVelocity);
                display.addData("浮板位置指令", "%.3f", appliedPanel);
                if (poseReady) {
                    turret.addTelemetry(display);
                    double testDistanceInches = calculateTestDistanceInches(current, target);
                    display.addData("距离对照（英寸）", "炮台 %.3f / 测试 %.3f",
                            distanceInches, testDistanceInches);
                    double difference = Math.abs(testDistanceInches - distanceInches);
                    if (Double.isFinite(difference)) {
                        display.addData("距离差值（英寸）", "%.4f", difference);
                    } else {
                        display.addData("距离差值（英寸）", "距离无效");
                    }
                } else {
                    display.addData("距离对照（英寸）", "等待定位");
                }
                display.addData("记录测点", recordStatus);
                display.addData("最近测点", lastSample);
                display.update();
                telemetryTimer.reset();
            }
        } finally {
            drivetrain.drive(gamepad1, 0.0);
            shooter.stop();
            intake.intakeStop();
            turret.setEnabled(false);
        }
    }

    private static double calculateTestDistanceInches(Pose2D robot, Pose2D target) {
        double forwardOffset = autoTurretAim.POLLEN_SHOOTER_DRIVETRAIN_OFFSET_MM / 25.4;
        double leftOffset = autoTurretAim.TURRET_LEFT_OFFSET_MM / 25.4;
        double heading = robot.getHeading(AngleUnit.RADIANS);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double turretX = robot.getX(DistanceUnit.INCH) + forwardOffset * cos - leftOffset * sin;
        double turretY = robot.getY(DistanceUnit.INCH) + forwardOffset * sin + leftOffset * cos;
        double dx = target.getX(DistanceUnit.INCH) - turretX;
        double dy = target.getY(DistanceUnit.INCH) - turretY;
        return Math.hypot(dx, dy);
    }

    private void recordSample(Pose2D target, double distanceInches, boolean flywheelRunning,
                              double targetVelocity, double measuredVelocity, double panelPosition) {
        if (!Double.isFinite(distanceInches) || !flywheelRunning
                || !Double.isFinite(measuredVelocity) || !Double.isFinite(panelPosition)) {
            recordStatus = "请确认 Pinpoint 就绪、飞轮已开启且 Panel 指令有效";
            return;
        }

        double targetX = target.getX(DistanceUnit.INCH);
        double targetY = target.getY(DistanceUnit.INCH);
        sampleCount++;
        lastSample = String.format(Locale.US,
                "#%d | 距离 %.2f in | 转速 %.0f / %.0f ticks/s | Panel %.3f | 靶点 (%.1f, %.1f)",
                sampleCount, distanceInches, targetVelocity, measuredVelocity, panelPosition, targetX, targetY);
        recordStatus = "已记录，请另记试投命中次数";

        RobotLog.ii("ShooterDistanceTest",
                "sample=%d distance_in=%.2f target_ticks_per_s=%.0f actual_ticks_per_s=%.0f panel=%.3f target_x_in=%.1f target_y_in=%.1f",
                sampleCount, distanceInches, targetVelocity, measuredVelocity, panelPosition, targetX, targetY);
    }
}
