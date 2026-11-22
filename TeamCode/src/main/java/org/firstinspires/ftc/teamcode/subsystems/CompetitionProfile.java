package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kernel.Kernel;
import org.firstinspires.ftc.teamcode.kernel.Subsystem;
import org.firstinspires.ftc.teamcode.kernel.profiles.RobotProfile;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Function;

public final class CompetitionProfile implements RobotProfile {
    private final Function<HardwareMap, Follower> autonomousFollower;

    private CompetitionProfile(Function<HardwareMap, Follower> autonomousFollower) {
        this.autonomousFollower = autonomousFollower;
    }

    public static CompetitionProfile teleOp() {
        return new CompetitionProfile(null);
    }

    public static CompetitionProfile autonomous(Function<HardwareMap, Follower> createFollower) {
        return new CompetitionProfile(createFollower);
    }

    @Override
    public Follower follower(HardwareMap hardwareMap) {
        return autonomousFollower == null ? null : autonomousFollower.apply(hardwareMap);
    }

    @Override
    public Map<Class<? extends Subsystem>, Subsystem> custom(Kernel kernel) {
        Map<Class<? extends Subsystem>, Subsystem> devices = new LinkedHashMap<>();
        if (autonomousFollower == null) {
            Drivetrain drive = new Drivetrain();
            drive.bind(kernel);
            devices.put(Drivetrain.class, drive);
        }

        Shooter shooter = new Shooter();
        shooter.bind(kernel);
        devices.put(Shooter.class, shooter);
        devices.put(autoTurretAim.class, new autoTurretAim(kernel));

        Intake intake = new Intake();
        intake.bind(kernel);
        devices.put(Intake.class, intake);
        return devices;
    }

    @Override
    public boolean manageBulkCaching() {
        return false;
    }

    @Override
    public boolean indicateAllianceOnGamepads() {
        return false;
    }
}
