Runtime services
================

Input and bindings
------------------

``robot.gp1`` and ``robot.gp2`` expose buttons, sticks, triggers, raw axes, and latched ``takePress(Key)`` calls. Annotation bindings such as ``@OnPress`` and ``@WhileHeld`` may be placed on OpMode methods. The OpMode is bound automatically; helpers need ``robot.bind(helper)``. Bindings can select a gamepad and modifier keys.

.. mermaid::

   flowchart TB
       accTitle: Runtime annotation binding
       accDescr: Robot binds the OpMode or a helper. Input delegates key annotations to AnnotationBinder. MemberBinder registers watches with telemetry and recurring methods with Scheduler.
       bind["Robot.bind(target)"] --> input["Input → AnnotationBinder"]
       bind --> member["MemberBinder"]
       input -->|"e.g. @OnPress, @WhileHeld"| keys["Input callbacks"]
       member -->|"@Watch"| telemetry["KernelTelemetry"]
       member -->|"@Every"| scheduler["Scheduler"]

Telemetry, timing, and events
-----------------------------

``@Watch`` exposes a field or zero-argument value-returning method in telemetry. ``@Every(milliseconds)`` schedules a zero-argument method after start by default; ``duringInit = true`` opts into init. Annotated methods must be non-static and non-private. ``robot.telemetry`` collects values for the status display; ``isDue()`` can avoid expensive formatting on every loop.

``robot.after`` and ``robot.every`` schedule callbacks, and ``robot.on`` subscribes to typed events. Their returned subscriptions can be cancelled. Callbacks are serviced by ticks, so keep them short.

Motion and pose
---------------

When a profile supplies a Pedro follower, ``robot.motion`` exposes driving, path following and queuing, turns, holds, aiming, pose, and completion events. It is ``null`` without a follower. ``Route`` builds straight or curved Pedro paths and interpolates heading between endpoint poses. Motion drive inputs are forward, left, and counterclockwise; headings in the motion API are radians. ``robot.await(robot.motion)`` waits until motion is settled while continuing to tick.

If autonomous reached start and has a Pedro follower, shutdown saves the final pose and alliance in the in-memory ``PoseStore``. A later TeleOp with motion can restore both automatically. ``CompetitionTeleOp`` reads the stored values through its own helpers.

.. mermaid::

   flowchart TB
       accTitle: Autonomous pose handoff to TeleOp
       accDescr: Robot shutdown saves pose and alliance after autonomous has started and motion is present. A TeleOp with motion can restore them automatically. CompetitionTeleOp reads the values directly to support its own drivetrain.
       auto["Robot.shutdown()<br/>Autonomous started with motion"] -->|saves pose and alliance| pose["PoseStore"]
       pose -->|resumeInto| motion["TeleOp with motion"]
       pose -->|"peek and peekAlliance"| tele["CompetitionTeleOp helpers"]
