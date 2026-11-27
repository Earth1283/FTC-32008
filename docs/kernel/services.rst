Runtime services
================

Input and bindings
------------------

``robot.gp1`` and ``robot.gp2`` expose buttons, sticks, triggers, raw axes, and latched ``takePress(Key)`` calls. Annotation bindings such as ``@OnPress`` and ``@WhileHeld`` may be placed on OpMode methods. The OpMode is bound automatically; helpers need ``robot.bind(helper)``. Bindings can select a gamepad and modifier keys.

Telemetry, timing, and events
-----------------------------

``@Watch`` exposes a field or zero-argument value-returning method in telemetry. ``@Every(milliseconds)`` schedules a zero-argument method after start by default; ``duringInit = true`` opts into init. Annotated methods must be non-static and non-private. ``robot.telemetry`` collects values for the status display; ``isDue()`` can avoid expensive formatting on every loop.

``robot.after`` and ``robot.every`` schedule callbacks, and ``robot.on`` subscribes to typed events. Their returned subscriptions can be cancelled. Callbacks are serviced by ticks, so keep them short.

Motion and pose
---------------

When a profile supplies a Pedro follower, ``robot.motion`` exposes driving, path following and queuing, turns, holds, aiming, pose, and completion events. It is ``null`` without a follower. ``Route`` builds straight or curved Pedro paths and interpolates heading between endpoint poses. Motion drive inputs are forward, left, and counterclockwise; headings in the motion API are radians. ``robot.await(robot.motion)`` waits until motion is settled while continuing to tick.

At shutdown after a started autonomous, the kernel stores the final Pedro pose and alliance in ``PoseStore``. This is an in-memory handoff to a later OpMode, not persistent storage.
