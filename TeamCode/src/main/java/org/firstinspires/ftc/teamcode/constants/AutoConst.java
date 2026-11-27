package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.geometry.Pose;

/** Pedro field positions use inches and headings use radians. */
public class AutoConst {
    // unit in milliseconds
    public static long SHOOTER_START_TIME = 2600;
    public static long SHOOTER_READY_TIME = 200;
    public static long EMPTY_MAGAZINE_TIME = 1200;
    public static long FLOWER_RELOAD_TIME = 200;

    public static Pose RED_START = new Pose(57, 6.7, Math.toRadians(180));
    public static Pose RED_START_FLOWER = new Pose(61, 134.5, Math.toRadians(180));
    public static Pose RED_INTAKE_GARDEN = new Pose(21, 6.7, Math.toRadians(180));
    public static Pose RED_INTAKE_GARDEN_2 = new Pose(9, 6.7, Math.toRadians(180));
    public static Pose RED_SHOOT_GARDEN = new Pose(37, 122, Math.toRadians(50));
    public static Pose RED_SHOOT_GARDEN_CONTROL_1 = new Pose(45, 20);
    public static Pose RED_SHOOT_GARDEN_CONTROL_2 = new Pose(22, 116);
    public static Pose RED_INTAKE_FLOWER_1 = new Pose(47.5, 127, Math.toRadians(90));
    public static Pose RED_INTAKE_FLOWER_1_FROM_PRELOAD_1 = new Pose(46.5, 110, Math.toRadians(90));
    public static Pose RED_INTAKE_FLOWER_1_FROM_PRELOAD_2 = new Pose(46.5, 127, Math.toRadians(90));
    public static Pose RED_INTAKE_TEAMMATE = new Pose(8.5, 108, Math.toRadians(-90));
    public static Pose RED_INTAKE_TEAMMATE_2 = new Pose(8.5, 97, Math.toRadians(-90));
    public static Pose RED_INTAKE_TEAMMATE_CONTROL_1 = new Pose(40, 110);
    public static Pose RED_SHOOT_TEAMMATE = new Pose(57, 14.5, Math.toRadians(-180));
    public static Pose RED_SHOOT_TEAMMATE_CONTROL_1 = new Pose(70, 128);
    public static Pose RED_SHOOT_TEAMMATE_CONTROL_2 = new Pose(0, 15);
    public static Pose RED_INTAKE_FLOWER_2 = new Pose(15, 46, Math.toRadians(180));
    public static Pose RED_INTAKE_FLOWER_2_CONTROL_FROM_GARDEN = new Pose(36, 47);
    public static Pose RED_INTAKE_FLOWER_2_CONTROL_FROM_FLOWER_1_1 = new Pose(21, 118);
    public static Pose RED_INTAKE_FLOWER_2_CONTROL_FROM_FLOWER_1_2 = new Pose(50, 44);
    public static Pose RED_SHOOT_FLOWER_2 = new Pose(57, 14.5, Math.toRadians(180));
    public static Pose RED_SHOOT_FLOWER_2_SURE = new Pose(57, 132, Math.toRadians(180));
    public static Pose RED_SHOOT_FLOWER_2_SURE_CONTROL = new Pose(20, 115);
    public static Pose RED_PARK = new Pose(9, 80, Math.toRadians(90));
    public static Pose RED_PARK_CONTROL = new Pose(30, 20);
    public static Pose RED_PARK_FINAL = new Pose(9, 95, Math.toRadians(90));
    public static Pose RED_PARK_SURE = new Pose(14, 95, Math.toRadians(90));
    public static Pose RED_PARK_FROM_FLOWER = new Pose(9, 120, 0);
    public static Pose RED_PARK_S = new Pose(33, 135, 0);
    public static Pose RED_PARK_M = new Pose(10, 132, 0);
    public static Pose RED_PARK_E = new Pose(10, 115, 0);

}
