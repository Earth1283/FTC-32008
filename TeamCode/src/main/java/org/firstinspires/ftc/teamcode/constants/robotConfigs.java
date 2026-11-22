package org.firstinspires.ftc.teamcode.constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public final class robotConfigs {
    private robotConfigs() { }

    public static final double BODY_LENGTH_INCH = 448.0 / 25.4;
    public static final double BODY_WIDTH_INCH = 408.0 / 25.4;
    public static final double CENTER_TO_FRONT_INCH = 26.5 / 2.54;
    public static final double CENTER_TO_LEFT_INCH = 20.4 / 2.54;
    public static final double CENTER_TO_RIGHT_INCH = BODY_WIDTH_INCH - CENTER_TO_LEFT_INCH;

    public static final String LEFT_FRONT = "lf";
    public static final String LEFT_BACK = "lb";
    public static final String RIGHT_FRONT = "rf";
    public static final String RIGHT_BACK = "rb";
    public static final String PIN_POINT = "pp";

    // TeleOp Pinpoint pod offsets in centimeters.
    public static final double PINPOINT_X_OFFSET_CM = -13.0;
    public static final double PINPOINT_Y_OFFSET_CM = -3.3;

    public static final String TRANS_MOTOR = "transmotor";
    public static final String INTAKE = "intake";
    public static final String UP_SHOOTER = "upshooter";
    public static final String DOWN_SHOOTER = "downshooter";

    public static final String LEFT_GATE = "leftgate";
    public static final String RIGHT_GATE = "rightgate";
    public static final String LEFT_TURRET = "leftturret";
    public static final String RIGHT_TURRET = "rightturret";
    public static final String PANEL = "panel";

    public static final String LEFT_FLOWER_GATE = "leftflowergate";
    public static final String RIGHT_FLOWER_GATE = "rightflowergate";

}
