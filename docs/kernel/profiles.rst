Profiles and subsystems
=======================

A ``RobotProfile`` constructs the implementation used for each standard
subsystem and may supply a Pedro follower. A factory returns ``null`` when
that subsystem is absent. Profiles can also return a class-keyed map of
custom subsystems; OpModes access those with ``robot.require(Type.class)``.
``require`` turns an absent subsystem into a profile configuration panic;
``has`` and ``get`` support optional dependencies.

``KernelOpMode.profile()`` defaults to ``Profiles.ACTIVE`` and can be
overridden. ``Robot`` installs the profile's subsystems, and ``Kernel``
provides their shared context. ``Kernel.device`` looks up a configured device
by type and name, and reports missing devices with the profile and configured
names.

A subsystem implements ``Subsystem``. Its ``init()`` returns an initialization
result; ``update()`` runs once per tick; ``report()`` contributes status;
and ``stop()`` releases or stops outputs. Startup checks cover hubs, battery,
the localizer when present, installed subsystems, and profile init tasks. A
critical failure aborts initialization.

The competition profile installs ``Shooter``, ``autoTurretAim``, and the
competition ``Intake`` as custom subsystems. TeleOp also installs
``Drivetrain``; autonomous instead supplies a Pedro follower. The generic
``kernel.subsystems.Intake`` and competition ``subsystems.Intake`` are
different types, so imports matter. ``CompetitionProfile`` disables
kernel-managed bulk caching and gamepad alliance lights.
