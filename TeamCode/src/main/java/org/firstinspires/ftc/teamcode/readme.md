# FTC 32008 TeamCode

Competition OpModes use [the FTC-32008 kernel](kernel/KernelOpMode.java) for startup,
device updates, telemetry, and shutdown. The [competition profile](subsystems/CompetitionProfile.java)
binds the robot hardware using the names in [robotConfigs.java](constants/robotConfigs.java).

## Autonomous

All seven routes use red alliance field coordinates. Positions are in inches and Java
headings are in radians. Check the starting pose on the Panels field view after INIT.

| Driver Station name | Starting pose (X, Y, heading) | Parking pose |
| --- | --- | --- |
| Flower - Park Only | (33, 135, 0°) | (10, 115, 0°) |
| Flower - Preload and Park | (33, 135, 0°) | (10, 115, 0°) |
| Flower - Preload, Collect and Park | (61, 134.5, 180°) | (9, 120, 0°) |
| Garden - 2 Tips Solo | (57, 6.7, 180°) | (9, 95, 90°) |
| Garden - 2 Tips Solo (Sure) | (57, 6.7, 180°) | (14, 95, 90°) |
| Garden - 3 Tips Cooperative | (57, 6.7, 180°) | (9, 95, 90°) |
| Garden - 3 Tips Solo | (57, 6.7, 180°) | (9, 95, 90°) |

Waypoints and shared timings live in [AutoConst.java](constants/AutoConst.java).
Shots use fixed flywheel speeds and turret angles relative to the chassis. A
`Double.NaN` angle marks a shot awaiting calibration. The route skips feeding
for that shot. The kernel saves the measured pose and alliance at shutdown for
TeleOp in the same Robot Controller process.

## TeleOp

- **TeleOp Robot Centric** and **TeleOp Field Centric** share the same controls. In Panels,
  set the starting pose and alliance during INIT or set `USE_CUSTOM_START_POSE` to
  false to resume from autonomous. `HEADLESS_FORWARD_DEG` sets the forward stick
  direction in field-centric mode.
- Gamepad 1 steers, collects with RT, ejects with LT, toggles the flywheel with LB,
  and feeds with RB after a 100 ms gate delay. Y raises the shovels and A lowers them.
- In both turret modes, D-pad up/down on either gamepad trims the measured flywheel
  baseline by 10 ticks/s. Gamepad 2 D-pad left/right trims the active target's aim
  by 2° and Y/A adjusts the panel.
- **tele** resumes from autonomous and uses motion-compensated turret aim.
  Gamepad 1 touchpad toggles robot-centric driving with the turret centered.
  Gamepad 2 LB toggles the shovels.

## Diagnostics

Use **Shooter Distance Test** to compare the turret's calculated distance with
an independent field calculation and see each flywheel's signed encoder speed,
power, and analog input voltages. Press gamepad 2 X after speed stabilizes to
record a sample. **Auto Turret Aim Test**, **Servo Position Test**,
**Shovel Position Test**, and **Pinpoint Auto Tuner** cover the related hardware.

## FTC SDK template

Welcome!

Use this module to add OpModes for your team's Robot Controller app. The instructions
below describe the FTC SDK workflow for adding new OpModes.

## Creating your own OpModes

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements:
 FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

### Naming of Samples

To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (in detail) in the sample_conventions.md file in this folder.

To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.

Robot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.
            It may be used to provide a common baseline driving OpMode, or
            to demonstrate how a particular sensor or concept can be used to navigate.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.  These OpModes may not produce a drivable robot.

After the prefix, other conventions will apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Robot class names are constructed as:     Robot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

 1) Locate the desired sample class in the Project/Android tree.

 2) Right click on the sample class and select "Copy"

 3) Expand the  TeamCode/java folder

 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

 5) You will be prompted for a class name for the copy.
    Choose something meaningful based on the purpose of this class.
    Start with a capital letter, and remember that there may be more similar classes later.

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed on the
Driver Station's OpMode list.

Each OpMode sample class begins with several lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.



## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

In some situations, you have multiple teams in your club and you want them to all share
a common code organization, with each being able to *see* the others code but each having
their own team module with their own code that they maintain themselves.

In this situation, you might wish to clone the TeamCode module, once for each of these teams.
Each of the clones would then appear along side each other in the Android Studio module list,
together with the FtcRobotController module (and the original TeamCode module).

Selective Team phones can then be programmed by selecting the desired Module from the pulldown list
prior to clicking to the green Run arrow.

Warning:  This is not for the inexperienced Software developer.
You will need to be comfortable with File manipulations and managing Android Studio Modules.
These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.
 
Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

Note: Some names start with "Team" and others start with "team".  This is intentional.

1)  Using your operating system file management tools, copy the whole "TeamCode"
    folder to a sibling folder with a corresponding new name, eg: "Team0417".

2)  In the new Team0417 folder, delete the TeamCode.iml file.

3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder
    to a matching name with a lowercase 'team' eg:  "team0417".

4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains
         package="org.firstinspires.ftc.teamcode"
    to be
         package="org.firstinspires.ftc.team0417"

5)  Add:    include ':Team0417' to the "/settings.gradle" file.
    
6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project""
