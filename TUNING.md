# Pedro Pathing Tuning Guide (Mecanum + Pinpoint + Foresight)

How to tune Pedro Pathing with the web-based tuner in this project.

## How the tuner works

- The tuning library (`com.pedropathing:tuning`) starts a web server on the Control Hub when the Robot Controller app starts.
  - UI: `http://192.168.43.1:10158` (port 10158)
  - Websocket: port 12649
- You use it from a laptop or phone joined to the Control Hub's Wi-Fi. There is no OpMode to select on the Driver Hub.
- When a step needs the robot to move, the library starts an OpMode called "Pedro Tuning" by itself.
- Each tuner finishes by printing values and ready-to-paste Java config.
- The Driver Hub keeps the robot connected and gives you the stop button. Keep it within reach.

## Where things live

| File | Purpose |
| --- | --- |
| `TeamCode/.../pedro/Tuning.java` | Registers tuners with `@Tuner` static methods |
| `TeamCode/.../pedro/Constants.java` | Holds the generated configs and `create()` |
| `TeamCode/.../pedro/procedures/` | The tuner implementations (`MecanumTuner`, `PinpointTuner`, `ForesightTuner`, `Tests`, ...) |

A `@Tuner` method must be `static`, take no arguments, and return exactly `Procedure`. Otherwise the scanner throws at startup.

## 1. Build and deploy

```
./gradlew assembleDebug     # build the APK only
./gradlew installDebug      # build and install (Control Hub connected via adb)
```

Connect with `adb connect 192.168.43.1:5555` over the Control Hub's Wi-Fi, or use USB. You can also run the `TeamCode` configuration from IntelliJ.

Then restart the Robot Controller app. The web server only starts when the app starts.

## 2. Open the tuner

1. Join the Control Hub's Wi-Fi on a laptop.
2. Open `http://192.168.43.1:10158`.
3. You should see "1. Mecanum Directions" and "2. Pinpoint".

If the list is empty, the `@Tuner` scan didn't register them. Check the Robot Controller logcat.

## 3. Mecanum Directions

1. Enter your four drivetrain motor names (as in the hardware config).
2. Each wheel spins in turn. Use the diagram to see which wheel it is and which way is forward.
3. Tell the tuner whether each wheel spun forward or reversed.
4. Copy the generated `drivetrainConfig`:

```java
public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
    c.frontLeftName.set("...");
    // ... names and directions for all four motors
});
```

Paste it into `Constants.java`.

## 4. Pinpoint

1. Follow the prompts: Pinpoint device name, pod type (or a custom pod scalar), pod offsets, and pod directions.
2. Copy the generated `localizerConfig` (a `PinpointConfig`) and paste it into `Constants.java`.

## 5. Enable Foresight and Tests

In `Tuning.java`, uncomment the `foresight()` and `tests()` methods. Add these imports:

- `ForesightTuner` and `Tests` (from `...pedro.procedures`)
- `Mecanum` (`com.pedropathing.revhub.drivetrains`)
- `PinpointLocalizer` (`com.pedropathing.revhub.localizers`)
- `Foresight` (`com.pedropathing.algorithm`)

`Tests` also reads `Constants.foresightConfig`, which doesn't exist yet. Leave `tests()` commented until step 6 is done.

Redeploy and restart the Robot Controller app.

## 6. Foresight

**Clear several feet of floor first. This tuner drives the robot hard.**

The tuner runs these steps in order:

1. Max forward and strafe velocity (you enter a drive distance, default 48 in).
2. Forward and strafe deceleration (you enter a target velocity, default 30).
3. Heading braking and heading kP.
4. Forward and strafe braking (distance must be at least 15 in, default 36).
5. Translational kP values and the coast/brake feedforward.

The tuner stops with a message if a step produces a NaN or non-positive value, so it doesn't drive off on bad data.

When it finishes, copy the generated `foresightConfig` into `Constants.java`.

## 7. Wire up `Constants.create()`

```java
public static Follower create(HardwareMap h) {
    return new Follower(
            new PinpointLocalizer(h, localizerConfig),
            new Mecanum(h, drivetrainConfig),
            new Foresight(foresightConfig));
}
```

## 8. Verify with Tests

Uncomment `tests()` in `Tuning.java`, redeploy, and run **Tests**. Available checks: Hold, Line, Curve, Interpolation, Localization, Odometry, Driving, Pose.

Start with Localization/Odometry to confirm the Pinpoint reports sensible poses, then Line and Curve (default distance 48 in) to confirm path following, then Hold.

## Troubleshooting

- **Page won't load:** confirm you're on the Control Hub's Wi-Fi and that the Robot Controller app restarted after install.
- **Tuner list empty:** the method is probably not `static`, takes parameters, or doesn't return `Procedure`. The scanner reports that in logcat.
- **App crashes at startup after enabling Foresight/Tests:** a config in `Constants.java` is missing. The `Tuning` lambdas read them, so every config must be set first.
- **Build fails with `aapt2` offline errors:** run Gradle online. The offline cache doesn't include it.
