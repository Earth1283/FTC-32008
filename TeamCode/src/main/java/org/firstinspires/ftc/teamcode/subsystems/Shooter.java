package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.RobotConstants.PANEL_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.PANEL_MIN_POSITION;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.DOWN_SHOOTER;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.PANEL;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.UP_SHOOTER;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.kernel.Kernel;
import org.firstinspires.ftc.teamcode.kernel.Subsystem;

import java.util.function.Function;

@Configurable
public class Shooter implements Subsystem {
    private DcMotorEx upShooter, downShooter;
    private Servo panel;

    public static double POLLEN_SHOOTER_P = 13.0;
    public static double POLLEN_SHOOTER_F = 14.0;

    // Polynomial coefficients map inches to ticks per second.
    public static double POLLEN_A = 0.0;
    public static double POLLEN_B = -0.1714;
    public static double POLLEN_C = 23.714;
    public static double POLLEN_D = 1482.3;

    public static double POLLEN_VELOCITY_MIN = 1200;
    public static double POLLEN_VELOCITY_MAX = 2800;

    // Replace these example panel calibration points with measured values.
    public static double PANEL_NEAR_DISTANCE_IN = 40.0;
    public static double PANEL_NEAR_POSITION = 0.30;
    public static double PANEL_FAR_DISTANCE_IN = 100.0;
    public static double PANEL_FAR_POSITION = 0.45;

    public static double POLLEN_KP = 0.00022;
    public static double POLLEN_KV = 0.00037;
    public static double POLLEN_KS = 0.165;

    public void init(HardwareMap hardwareMap) {
        bind(name -> hardwareMap.get(DcMotorEx.class, name), hardwareMap.get(Servo.class, PANEL));
    }

    void bind(Kernel kernel) {
        bind(name -> kernel.device(DcMotorEx.class, name), kernel.device(Servo.class, PANEL));
    }

    private void bind(Function<String, DcMotorEx> motor, Servo panel) {
        upShooter = motor.apply(UP_SHOOTER);
        downShooter = motor.apply(DOWN_SHOOTER);
        this.panel = panel;

        upShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        downShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        downShooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPollenVelocity(double velocity) {
        upShooter.setVelocityPIDFCoefficients(POLLEN_SHOOTER_P, 0.0, 0.0, POLLEN_SHOOTER_F);
        upShooter.setVelocity(velocity);
        downShooter.setVelocityPIDFCoefficients(POLLEN_SHOOTER_P, 0.0, 0.0, POLLEN_SHOOTER_F);
        downShooter.setVelocity(velocity);
    }

    public void setPollenVelocityByDistance(double distance) {
        double targetVelocity = getPollenVelocityForDistance(distance, 0.0);
        if (!Double.isFinite(targetVelocity)) {
            stop();
            return;
        }
        setPollenMotorVelocity(targetVelocity);
    }

    /** Call each loop to update flywheel feedback. Invalid inputs stop the flywheel and hold the panel. */
    public boolean setPollenAndPanelByDistance(double distanceInches) {
        double panelPosition = getPanelPositionForDistance(distanceInches);
        double targetVelocity = getPollenVelocityForDistance(distanceInches, 0.0);
        double measuredVelocity = getPollenVelocity();
        if (!Double.isFinite(panelPosition) || !Double.isFinite(targetVelocity)
                || targetVelocity <= 0.0 || !Double.isFinite(measuredVelocity)) {
            stop();
            return false;
        }

        panel.setPosition(panelPosition);
        setPollenMotorVelocity(targetVelocity, measuredVelocity);
        return true;
    }

    public double getPanelPositionForDistance(double distanceInches) {
        double nearDistance = PANEL_NEAR_DISTANCE_IN;
        double farDistance = PANEL_FAR_DISTANCE_IN;
        double nearPosition = PANEL_NEAR_POSITION;
        double farPosition = PANEL_FAR_POSITION;
        if (!Double.isFinite(distanceInches) || distanceInches < 0.0
                || !Double.isFinite(nearDistance) || nearDistance < 0.0
                || !Double.isFinite(farDistance) || farDistance <= nearDistance
                || !Double.isFinite(nearPosition) || !Double.isFinite(farPosition)) {
            return Double.NaN;
        }

        double fraction = Range.clip((distanceInches - nearDistance) / (farDistance - nearDistance), 0.0, 1.0);
        double position = nearPosition * (1.0 - fraction) + farPosition * fraction;
        return limitPanelPosition(position);
    }

    public double getPollenVelocityForDistance(double distance, double velocityTrim) {
        double min = POLLEN_VELOCITY_MIN;
        double max = POLLEN_VELOCITY_MAX;
        double target = f(POLLEN_A, POLLEN_B, POLLEN_C, POLLEN_D, distance) + velocityTrim;
        if (!Double.isFinite(target) || !Double.isFinite(min) || !Double.isFinite(max)
                || min < 0.0 || min > max) {
            return Double.NaN;
        }
        return Range.clip(target, min, max);
    }

    public double f(double a, double b, double c, double d, double x) {
        return a * Math.pow(x, 3) + b * Math.pow(x, 2) + c * x + d;
    }

    public void setPollenPower(double power) {
        upShooter.setPower(power);
        downShooter.setPower(power);
    }

    public double getPollenVelocity(){
        return Math.abs(upShooter.getVelocity() + downShooter.getVelocity()) / 2.0;
    }

    public double getUpVelocity(){
        return upShooter.getVelocity();
    }

    public double getDownVelocity(){
        return downShooter.getVelocity();
    }

    public void setPollenMotorVelocity(double target) {
        setPollenMotorVelocity(target, getPollenVelocity());
    }

    public void setPollenMotorVelocity(double target, double measuredVelocity) {
        double error = target - measuredVelocity;
        setPollenPower(POLLEN_KS + target * POLLEN_KV + error * POLLEN_KP);
    }

    @Override
    public void stop() {
        upShooter.setPower(0);
        downShooter.setPower(0);
    }

    public void setPollenStop() {
        stop();
    }

    /** Returns the applied position or NaN if the panel command was skipped. */
    public double setPanel(double position) {
        double limitedPosition = limitPanelPosition(position);
        if (Double.isFinite(limitedPosition)) {
            panel.setPosition(limitedPosition);
        }
        return limitedPosition;
    }

    public static double limitPanelPosition(double position) {
        double min = PANEL_MIN_POSITION;
        double max = PANEL_MAX_POSITION;
        if (!Double.isFinite(position) || !Double.isFinite(min) || !Double.isFinite(max)
                || min < 0.0 || max > 1.0 || min > max) {
            return Double.NaN;
        }
        return Range.clip(position, min, max);
    }
}
