package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.RobotConstants.*;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kernel.Kernel;
import org.firstinspires.ftc.teamcode.kernel.Subsystem;

import java.util.function.Function;

@Configurable
public class Intake implements Subsystem {
    private DcMotorEx intake, trans;
    private Servo leftShovel, rightShovel, leftGate, rightGate;
    public static double feedingPower = 1.0;

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
        trans.setPower(feedingPower);
        openGate();
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
        leftGate.setPosition(LEFT_GATE_CLOSED);
        rightGate.setPosition(RIGHT_GATE_CLOSED);
    }
}
