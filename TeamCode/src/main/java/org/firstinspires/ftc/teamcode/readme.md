# Team Code OpModes

This directory contains the Autonomous and TeleOp modes for the robot, developed by the team.

## Autonomous OpModes

These OpModes are located in the `auto` package and utilize the `FollowerSubsystem` for path following.

### Red Alliance

*   **`Auto_RED_CLOSE_12`**
    *   **Start Position:** Close to the backdrop (Red side).
    *   **Routine:**
        1.  Shoots preloaded rings.
        2.  Intakes and shoots from the first row of the stack.
        3.  Intakes and shoots from the second row.
        4.  Intakes and shoots from the third row.
        5.  Parks.
    *   **Key Features:** High-scoring auto aiming for ~12 rings (preload + 3 stacks). Uses precise point-to-point navigation.

*   **`Auto_RED_FAR_6_BESIDE`**
    *   **Start Position:** Far from the backdrop (Red side).
    *   **Routine:**
        1.  Shoots preloaded rings.
        2.  Navigates to the stack to intake.
        3.  Performs multiple intake/shoot cycles (aiming for ~6 rings total).
        4.  Parks.

### Blue Alliance

*   **`Auto_BLUE_CLOSE_12`**
    *   **Start Position:** Close to the backdrop (Blue side).
    *   **Routine:** Mirrored version of `Auto_RED_CLOSE_12`. Shoots preload, then intakes and shoots 3 rows of stacks.

*   **`Auto_BLUE_FAR_6_BESIDE`**
    *   **Start Position:** Far from the backdrop (Blue side).
    *   **Routine:** Mirrored version of `Auto_RED_FAR_6_BESIDE`. Shoots preload, then performs intake/shoot cycles.

## TeleOp OpModes

These OpModes are located in the `teleOps` package.

### Main TeleOp

*   **`A_Tele`**
    *   **Description:** The primary TeleOp mode for competition.
    *   **Controls:**
        *   **Drive:** Manual driving (Gamepad 1).
        *   **Intake:** Triggers for intake in/out (Gamepad 1 & 2).
        *   **Shooter:** Buttons for preset shooter velocities/positions (Close: X, 80%: Y, Far: B) and manual adjustments via D-pad.
        *   **Transfer:** Bumpers for transfer control.

*   **`A_Tele_Single`**
    *   **Description:** Single-driver TeleOp mode where all critical controls are mapped to Gamepad 1. Useful for practice or solo testing.

### Advanced / Testing TeleOp

*   **`A_Tele_AutoVel_AutoRotate`**
    *   **Description:** Enhanced TeleOp with automatic velocity control and chassis auto-rotation capabilities.
    *   **Features:**
        *   Uses `PinPoint` odometry for position tracking.
        *   Automatically calculates distance to target for shooter velocity adjustment.
        *   `chasisAutoAim` toggle on Gamepad 1 (B button).

*   **`A_Tele_Blue_AutoVel_Train_exe` / `A_Tele_RED_AutoVel_Train_exe`**
    *   **Description:** Specialized TeleOp modes used for training or tuning automatic velocity calculations for specific alliances.
    *   **Features:** Includes specific target coordinates and continuous distance calculation logic.

*   **`Debug_Tele`**
    *   **Description:** A TeleOp mode focused on debugging hardware and tuning values.
    *   **Features:** detailed telemetry data for shooter velocity, current, position, and controller inputs.

## Test OpModes

Located in the `tests` package, these are for testing individual components:
*   `SensorGoBildaPinpointTest`: Test for the GoBilda Pinpoint odometry sensor.
*   `ServoPositionTest`: Utility to find correct servo positions.
*   `MotorDirectionTest`, `VelocityTest`: For motor tuning and verification.
*   `PIDTuner`: For tuning PID coefficients.
