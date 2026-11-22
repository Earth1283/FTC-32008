package org.firstinspires.ftc.teamcode.kernel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kernel.errors.OpModeStoppedException;
import org.firstinspires.ftc.teamcode.kernel.profiles.Profiles;
import org.firstinspires.ftc.teamcode.kernel.profiles.RobotProfile;

public abstract class KernelOpMode extends LinearOpMode {
    protected Robot robot;

    protected RobotProfile profile() {
        return Profiles.ACTIVE;
    }

    protected Telemetry display() {
        return telemetry;
    }

    protected void onInit() throws InterruptedException {}
    protected void onInitLoop() {}
    protected void onStart() throws InterruptedException {}
    protected void onLoop() {}
    protected void onStop() {}

    @Override
    public final void runOpMode() throws InterruptedException {
        robot = new Robot(this, profile(), display());
        try {
            onInit();
            robot.awaitStart(this::onInitLoop);
            if (isStopRequested()) return;
            robot.start();
            onStart();
            while (opModeIsActive()) robot.tick(this::onLoop);
        } catch (OpModeStoppedException stopped) {
            if (!isStopRequested()) throw stopped;
        } finally {
            try {
                onStop();
            } finally {
                robot.shutdown();
            }
        }
    }

    @Override
    public final void waitForStart() {
        try {
            robot.awaitStart(this::onInitLoop);
        } catch (OpModeStoppedException stopped) {
            Thread.currentThread().interrupt();
        }
    }
}
