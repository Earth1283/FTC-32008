Lifecycle and ticks
===================

``KernelOpMode`` extends the SDK's ``LinearOpMode`` and supplies five hooks:
``onInit``, ``onInitLoop``, ``onStart``, ``onLoop``, and ``onStop``. Its final ``runOpMode`` constructs ``Robot`` before ``onInit``, ticks while waiting for start, starts the match clock, then ticks with ``onLoop`` while the OpMode is active. The ``finally`` block calls ``onStop`` and shuts down
the robot, including when a hook fails after construction succeeds.

.. mermaid::

   sequenceDiagram
       accTitle: KernelOpMode lifecycle
       accDescr: Robot is constructed before any OpMode hook runs. Init ticks call onInitLoop. Active ticks call onLoop. The stop hook and robot shutdown run from nested finally blocks.
       participant Mode as KernelOpMode
       participant Robot
       Mode->>Robot: Construct and run boot checks
       Mode->>Mode: onInit()
       Mode->>Robot: awaitStart(onInitLoop)
       loop Waiting for start
           Robot->>Robot: tick(onInitLoop)
           Robot->>Mode: onInitLoop()
       end
       opt No stop requested
           Mode->>Robot: start() starts match clock
           Mode->>Mode: onStart()
           loop OpMode is active
               Mode->>Robot: tick(onLoop)
               Robot->>Mode: onLoop()
           end
       end
       Mode->>Mode: onStop() in finally
       Mode->>Robot: shutdown() in nested finally

Tick order
----------

.. mermaid::

   flowchart TB
       accTitle: Order of operations inside Robot.tick
       accDescr: A tick marks timing, clears managed caches, and samples input. Started ticks also arm members and fire input bindings. Motion, subsystems, the OpMode callback, scheduling, the match clock, event dispatch, and due telemetry follow in order.
       timing["Mark timing and clear managed hub caches"] --> sample["Sample input"]
       sample --> started{"Robot started?"}
       started -->|Yes| bindings["Arm pending @Every methods and fire input bindings"]
       started -->|No| update["Update PedroMotion if present,<br/>then installed subsystems"]
       bindings --> update
       update --> callback["Run OpMode callback if supplied"]
       callback --> clock["Update Scheduler, then MatchClock"]
       clock --> events["Dispatch queued events"]
       events --> due{"Telemetry due?"}
       due -->|Yes| status["Render StatusPanel and publish telemetry"]
       due -->|No| done(["End tick"])
       status --> done

A tick runs in this order:

1. Mark loop timing and clear managed hub caches.
2. Sample input; after start, arm scheduled members and fire input bindings.
3. Update Pedro motion, if present, then each installed subsystem.
4. Run the supplied OpMode loop callback.
5. Update the scheduler and match clock, then dispatch queued events.
6. Publish the status display when telemetry is due.

Input is sampled during init, but bindings are disarmed until start. A runtime exception in a tick stops outputs and reports a kernel panic. Subsystem ``stop()`` implementations should be idempotent and must not throw.

``robot.awaitStart()``, ``waitUntil``, ``sleep``, and ``await`` keep ticking while waiting in a linear OpMode. Do not call these waiting methods from a loop callback or another tick: the kernel detects reentrant ticks. A ``KernelOpMode`` already handles waiting and ticking, so put work in its hooks. A plain SDK OpMode can instead construct ``Robot`` and must arrange its own calls to ``tick``.
