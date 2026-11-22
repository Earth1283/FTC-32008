package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.kernel.Kernel;
import org.firstinspires.ftc.teamcode.kernel.Subsystem;

import java.util.function.Function;

@Configurable
public class Drivetrain implements Subsystem {
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private GoBildaPinpointDriver pinpoint;
    private double theta, power, turn, realTheta;

    public void init(HardwareMap hardwareMap) {
        bind(hardwareMap.get(GoBildaPinpointDriver.class, PIN_POINT), name -> hardwareMap.get(DcMotorEx.class, name));
    }

    void bind(Kernel kernel) {
        bind(kernel.device(GoBildaPinpointDriver.class, PIN_POINT), name -> kernel.device(DcMotorEx.class, name));
    }

    private void bind(GoBildaPinpointDriver pinpoint, Function<String, DcMotorEx> motor) {
        this.pinpoint = pinpoint;
        leftFront = motor.apply(LEFT_FRONT);
        leftBack = motor.apply(LEFT_BACK);
        rightFront = motor.apply(RIGHT_FRONT);
        rightBack = motor.apply(RIGHT_BACK);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pinpoint.setOffsets(PINPOINT_X_OFFSET_CM, PINPOINT_Y_OFFSET_CM, DistanceUnit.CM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

    }

    public void drive(Gamepad gamepad, double powerScale) {
        drive(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x, powerScale);
    }

    public void drive(double forward, double strafeRight, double yaw, double powerScale) {
        double rx = yaw * 0.85;
        double y = forward;
        double x = strafeRight;
        leftFront.setPower((y + x + rx) * powerScale);
        leftBack.setPower((y - x + rx) * powerScale);
        rightFront.setPower((y - x - rx) * powerScale);
        rightBack.setPower((y + x - rx) * powerScale);
    }

    public double getHeading() {
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    public void driveFieldOriented(Gamepad gamepad) {
        double y = -gamepad.left_stick_y, x = gamepad.left_stick_x, rx = gamepad.right_stick_x * 0.9;
        theta = Math.atan2(y, x) * 180 / Math.PI;
        power = Math.hypot(x, y);
        turn = rx;

        realTheta = (360 - pinpoint.getPosition().getHeading(AngleUnit.DEGREES)) + theta;

        double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = (power * cos / maxSinCos + turn);
        double rightFrontPower = (power * sin / maxSinCos - turn);
        double leftBackPower = (power * sin / maxSinCos + turn);
        double rightBackPower = (power * cos / maxSinCos - turn);

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    public void driveFieldOriented(double forward, double strafeRight, double yaw, double headingOffsetDeg) {
        double y = forward;
        double x = strafeRight;
        double rx = yaw * 0.85;
        double theta = Math.toDegrees(Math.atan2(y, x));
        double power = Math.hypot(x, y);

        double currentHeading = (pinpoint.getHeading(AngleUnit.RADIANS) + Math.PI
                - Math.toRadians(headingOffsetDeg)) % (Math.PI * 2) - Math.PI;
        double realTheta = (360 - Math.toDegrees(currentHeading)) + theta;

        double sin = Math.sin(Math.toRadians(realTheta) - (Math.PI / 4));
        double cos = Math.cos(Math.toRadians(realTheta) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

        leftFront.setPower(power * cos / maxSinCos + rx);
        leftBack.setPower(power * sin / maxSinCos + rx);
        rightFront.setPower(power * sin / maxSinCos - rx);
        rightBack.setPower(power * cos / maxSinCos - rx);
    }

    public Pose2D getPosition() {
        return pinpoint.getPosition();
    }

    public void setPosition(Pose2D pose) {
        pinpoint.setPosition(pose);
    }

    public DeviceStatus getLocalizationStatus() {
        return pinpoint.getDeviceStatus();
    }

    public double getVelocityXInchesPerSecond() {
        return pinpoint.getVelX(DistanceUnit.INCH);
    }

    public double getVelocityYInchesPerSecond() {
        return pinpoint.getVelY(DistanceUnit.INCH);
    }

    @Override
    public void update() {
        pinpoint.update();
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
