package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.RobotConstants.*;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kernel.Kernel;
import org.firstinspires.ftc.teamcode.kernel.Subsystem;

import java.util.function.Function;

@Configurable
public class Intake implements Subsystem {
    private DcMotorEx intake, trans;
    private Servo leftShovel, rightShovel, leftGate, rightGate;
    private static final double FEED_GATE_DELAY_MS = 100.0;
    public static double feedingPower = 0.8;
    private final ElapsedTime feedGateTimer = new ElapsedTime();
    private boolean feedGateOpened;

    public void init(HardwareMap hardwareMap) {
        bind(name -> hardwareMap.get(DcMotorEx.class, name), name -> hardwareMap.get(Servo.class, name));
    }

    void bind(Kernel kernel) {
        bind(name -> kernel.device(DcMotorEx.class, name), name -> kernel.device(Servo.class, name));
    }

    private void bind(Function<String, DcMotorEx> motor, Function<String, Servo> servo) {
        intake = motor.apply(INTAKE);
        trans = motor.apply(TRANS_MOTOR);

        leftShovel = servo.apply(LEFT_FLOWER_GATE);
        rightShovel = servo.apply(RIGHT_FLOWER_GATE);

        leftGate = servo.apply(LEFT_GATE);
        rightGate = servo.apply(RIGHT_GATE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        trans.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void intakeIn() {
        intake.setPower(1);
        trans.setPower(1);
        closeGate();
    }

    public void intakeOut() {
        feedGateOpened = false;
        intake.setPower(-1);
        trans.setPower(-1);
        openGate();
    }

    public void intakeStop() {
        intake.setPower(0);
        trans.setPower(0);
        closeGate();
    }

    public void intakeFeed() {
        intake.setPower(feedingPower);
        trans.setPower(1.0);
        openGate();
    }

    /** Call while the feed button is held to give the gate time to open. */
    public void intakeFeedWithGateDelay() {
        if (!feedGateOpened) {
            intake.setPower(0);
            trans.setPower(0);
            openGate();
            feedGateTimer.reset();
            feedGateOpened = true;
        }
        if (feedGateTimer.milliseconds() >= FEED_GATE_DELAY_MS) intakeFeed();
    }

    public void shovelDown() {
        leftShovel.setPosition(L_SHOVEL_DOWN);
        rightShovel.setPosition(R_SHOVEL_DOWN);
    }

    public void shovelUp() {
        leftShovel.setPosition(L_SHOVEL_UP);
        rightShovel.setPosition(R_SHOVEL_UP);
    }

    @Override
    public void stop() {
        intakeStop();
    }

    public void openGate() {
        leftGate.setPosition(LEFT_GATE_OPEN);
        rightGate.setPosition(RIGHT_GATE_OPEN);
    }

    public void closeGate() {
        feedGateOpened = false;
        leftGate.setPosition(LEFT_GATE_CLOSED);
        rightGate.setPosition(RIGHT_GATE_CLOSED);
    }
}
