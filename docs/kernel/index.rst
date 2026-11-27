Kernel architecture
===================

The kernel is the runtime layer in ``TeamCode`` that sits in between an FTC OpMode and our robot-specific code. It gives OpModes a consistent lifecycle, installs subsystems via a profile, and coordinates services such as input, motion, telemetry, timing, and events. Its source code lives in ``TeamCode/src/main/java/org/firstinspires/ftc/teamcode/kernel``.

.. mermaid::

   flowchart TB
       accTitle: Kernel runtime overview
       accDescr: Competition OpModes extend KernelOpMode, which creates and drives Robot. Robot uses a profile to install subsystems and runs the runtime services. Kernel supplies the subsystem registry and hardware access.
       team["CompetitionAuto / CompetitionTeleOp"] -->|extend| base["KernelOpMode<br/>extends SDK LinearOpMode"]
       base -->|creates and drives| robot["Robot"]
       robot -->|uses factories from| profile["RobotProfile"]
       robot -->|owns| kernel["Kernel"]
       robot -->|coordinates| services["Input, motion, telemetry,<br/>clocks, scheduler, events"]
       profile -->|creates| subsystem["Subsystem implementations"]
       kernel -->|registers by type| subsystem
       subsystem -->|uses shared context| kernel
       kernel -->|looks up devices| hardware["FTC HardwareMap"]

The main pieces have distinct responsibilities:

* ``KernelOpMode`` adapts the FTC ``LinearOpMode`` lifecycle into hooks.
* ``Robot`` owns the runtime and coordinates each tick and shutdown.
* ``Kernel`` provides subsystem registration, device lookup, event emission, timing, and optional hub cache management to subsystem implementations.
* ``RobotProfile`` supplies the follower and subsystem implementations for a particular configuration.
* ``Subsystem`` defines initialization, update, reporting, and stop hooks.

Competition OpModes and subsystems live outside the kernel package. For instance, ``CompetitionProfile`` connects the kernel to the competition implementation; ``CompetitionTeleOp`` and ``CompetitionAuto`` provide shared behavior for their respective OpModes.

.. toctree::
   :maxdepth: 2

   lifecycle
   profiles
   services
