Kernel architecture
===================

The kernel is the runtime layer in ``TeamCode`` that sits in between an FTC OpMode and our robot-specific code. It gives OpModes a consistent lifecycle, installs subsystems via a profile, and coordinates services such as input, motion, telemetry, timing, and events. Its source code lives in ``TeamCode/src/main/java/org/firstinspires/ftc/teamcode/kernel``.


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
