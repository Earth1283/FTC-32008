package org.firstinspires.ftc.teamcode.kernel;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;

import com.pedropathing.geometry.Pose;

import org.junit.Test;

public class AlliancesAndAnglesTest {
    private static final double EPS = 1e-9;

    @Test
    public void wrapKeepsAnglesInHalfOpenRange() {
        assertEquals(0, Angles.wrap(2 * Math.PI), EPS);
        assertEquals(-Math.PI, Angles.wrap(Math.PI), EPS);
        assertEquals(Math.PI / 2, Angles.wrap(-3 * Math.PI / 2), EPS);
        assertEquals(-Math.PI / 2, Angles.wrap(7 * Math.PI / 2), EPS);
    }

    @Test
    public void errorTakesTheShortWayAround() {
        assertEquals(Math.toRadians(20), Angles.error(Math.toRadians(350), Math.toRadians(10)), EPS);
        assertEquals(Math.toRadians(-20), Angles.error(Math.toRadians(10), Math.toRadians(350)), EPS);
    }

    @Test
    public void towardPointsAtTheTarget() {
        assertEquals(Math.PI / 2, Angles.toward(0, 0, 0, 5), EPS);
        assertEquals(Math.PI / 4, Angles.toward(1, 1, 2, 2), EPS);
    }

    @Test
    public void blueIsIdentity() {
        Pose pose = new Pose(10, 20, 0.5);
        assertSame(pose, Alliance.BLUE.pose(pose));
        assertEquals(0.5, Alliance.BLUE.heading(0.5), EPS);
    }

    @Test
    public void redMirrorsAcrossTheCenterline() {
        Pose red = Alliance.RED.pose(10, 20, 0);
        assertEquals(134, red.getX(), EPS);
        assertEquals(20, red.getY(), EPS);
        assertEquals(0, Angles.error(red.getHeading(), Math.PI), EPS);
        Pose point = Alliance.RED.point(30, 40);
        assertEquals(114, point.getX(), EPS);
        assertEquals(40, point.getY(), EPS);
    }

    @Test
    public void mirrorIsAnInvolution() {
        Pose original = new Pose(33, 71, 1.1);
        Pose twice = Alliance.mirror(Alliance.mirror(original));
        assertEquals(original.getX(), twice.getX(), EPS);
        assertEquals(original.getY(), twice.getY(), EPS);
        assertEquals(0, Angles.error(original.getHeading(), twice.getHeading()), EPS);
    }
}
