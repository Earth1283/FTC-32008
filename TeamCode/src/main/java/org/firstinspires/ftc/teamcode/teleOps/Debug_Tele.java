package org.firstinspires.ftc.teamcode.teleOps;

import static org.firstinspires.ftc.teamcode.constants.RobotConstants.autoEndH;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.autoEndX;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.autoEndY;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.teleOpTargetX;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.teleOpTargetY;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Debug Tele", group="Debug")
public class Debug_Tele extends LinearOpMode {
    Robot robot = new Robot();
    double targetX = 140, targetY = 140;
    
    // Variables from A_Tele_AutoVel_AutoRotate
    int turretTargetHeading = 0;
    double targetATAN, turretCurrentHeading;

    boolean shooterOn = false;
    double distance;
    JoinedTelemetry joinedTele;
    boolean chasisAutoAim = false;

    double distanceOffset = 0;
    int velo = 1500;
    double velocityCorrection = 1350;
    ElapsedTime timer = new ElapsedTime();
    double panelPos = .85;

    boolean intakeStateStop1 = false, intakeStateStop2 = false, intakeOut1State = false, intakeOut2State = false;

    ScheduledExecutorService exec = Executors.newScheduledThreadPool(50);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        // Initialization logic from A_Tele_AutoVel_AutoRotate
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        targetX = teleOpTargetX;
        targetY = teleOpTargetY;
        
        waitForStart();
        
        // Reset logic from A_Tele_AutoVel_AutoRotate
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));

        while (opModeIsActive()) {
            // Drive logic from A_Tele_AutoVel_AutoRotate
            robot.drivetrain.driveConstantOriented(gamepad1, chasisAutoAim);
            
            // Calc distance/position
            Pose2D current = robot.drivetrain.getPosition();
            distance = Math.abs(Math.hypot(targetY - current.getY(DistanceUnit.INCH), targetX - current.getX(DistanceUnit.INCH)));

            // Intake Logic - Gamepad 1 (Mirrored)
            if (gamepad1.left_trigger > 0) {
                robot.intake.intakeOut();
                shooterOn = false;
                intakeStateStop1 = false;
                intakeOut1State = false;
            }
            else if (gamepad1.right_trigger > 0) {
                robot.intake.intakeIn();
                intakeOut1State = false;
            } else {
                intakeStateStop1 = true;
                intakeOut1State = true;
            }

            // Intake/Transfer Logic - Gamepad 2 (Mirrored)
            if (gamepad2.right_bumper) {
                 intakeStateStop2 = false;
                 robot.intake.intakeIn();
                 robot.intake.transferToShooterUP();
            }
            else if (gamepad2.right_trigger > 0) {
                robot.intake.intakeOut();
                intakeOut2State = false;
                robot.intake.transferToShooterDown();
            } else {
                intakeStateStop2 = true;
                intakeOut2State = true;
                robot.intake.transferToShooterStop();
            }
            
            if (intakeStateStop1 && intakeStateStop2 && intakeOut1State && intakeOut2State) {
                robot.intake.intakeStop();
            }

            // Shooter Presets (Mirrored)
            if (gamepad2.xWasPressed()) {
                velocityCorrection = 1850;
                panelPos = 0.78;
            }

            if (gamepad2.bWasPressed()) {
                velocityCorrection = 1350;
                panelPos = 0.85;
            }

            // Manual Adjustments (Mirrored)
            if (gamepad2.dpadUpWasPressed()) velocityCorrection += 50;
            if (gamepad2.dpadDownWasPressed()) velocityCorrection -= 50;
            if (gamepad2.dpadLeftWasPressed()) panelPos -= 0.01;
            if (gamepad2.dpadRightWasPressed()) panelPos += 0.01;
            
            if (panelPos < 0) panelPos = 0;
            if (panelPos > 1) panelPos = 1;

            // Shooter Toggle (Mirrored)
            if (gamepad2.leftBumperWasPressed()) {
                exec.schedule(() -> shooterOn = !shooterOn, 200, TimeUnit.MILLISECONDS);
                timer.reset();
            }

            if (shooterOn) {
                robot.shooter.setShooter(panelPos, velocityCorrection);
            } else {
                robot.shooter.shooterStop();
            }

            // Chassis Auto Aim Toggle (Mirrored)
            if (gamepad1.bWasPressed()) {
                chasisAutoAim = !chasisAutoAim;
                gamepad1.rumbleBlips(1);
            }

            // --- ADVANCED TELEMETRY (DEBUG MODE FEATURE) ---
            telemetry.addLine("=== SHOOTER DEBUG ===");
            telemetry.addData("Shooter ON", shooterOn);
            telemetry.addData("Target Velocity", velocityCorrection);
            telemetry.addData("Actual Velocity L", robot.shooter.leftShooter.getVelocity());
            telemetry.addData("Actual Velocity R", robot.shooter.rightShooter.getVelocity());
            telemetry.addData("Panel Position", "%.3f", panelPos);

            telemetry.addLine("\n=== POSITION DEBUG ===");
            telemetry.addData("X (in)", "%.2f", current.getX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", "%.2f", current.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", "%.2f°", current.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Distance to Target", "%.2f", distance);
            telemetry.addData("Chassis Auto Aim", chasisAutoAim);

            telemetry.addLine("\n=== DRIVE DEBUG ===");
            telemetry.addData("Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Stick X", gamepad1.left_stick_x);
            telemetry.addData("Stick RX", gamepad1.right_stick_x);

            joinedTele.update();
            robot.drivetrain.pinPoint.update();
        }
        robot.shooter.shooterStop();
    }
}