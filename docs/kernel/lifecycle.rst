Lifecycle and ticks
===================

``KernelOpMode`` extends the SDK's ``LinearOpMode`` and supplies five hooks:
``onInit``, ``onInitLoop``, ``onStart``, ``onLoop``, and ``onStop``. Its final ``runOpMode`` constructs ``Robot`` before ``onInit``, ticks while waiting for start, starts the match clock, then ticks with ``onLoop`` while the OpMode is active. The ``finally`` block calls ``onStop`` and shuts down
the robot, including when a hook fails.

A tick runs in this order:

1. Mark loop timing and clear managed hub caches.
2. Sample input; after start, arm scheduled members and fire input bindings.
3. Update Pedro motion, if present, then each installed subsystem.
4. Run the supplied OpMode loop callback.
5. Update the scheduler and match clock, then dispatch queued events.
6. Publish the status display when telemetry is due.

Input is sampled during init, but bindings are disarmed until start. A runtime exception in a tick stops outputs and reports a kernel panic. Subsystem ``stop()`` implementations should be idempotent and must not throw.

``robot.awaitStart()``, ``waitUntil``, ``sleep``, and ``await`` keep ticking while waiting in a linear OpMode. Do not call these waiting methods from a loop callback or another tick: the kernel detects reentrant ticks. A ``KernelOpMode`` already handles waiting and ticking, so put work in its hooks. A plain SDK OpMode can instead construct ``Robot`` and must arrange its own calls to ``tick``.
