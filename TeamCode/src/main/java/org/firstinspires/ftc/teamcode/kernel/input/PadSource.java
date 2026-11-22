package org.firstinspires.ftc.teamcode.kernel.input;

import java.util.function.BooleanSupplier;

public interface PadSource {
    boolean isDown(Key key);

    double axis(PadAxis axis);

    default boolean takePress(Key key, BooleanSupplier sampledPress) {
        return sampledPress.getAsBoolean();
    }

    default boolean connected() {
        return true;
    }

    default void rumble(double left, double right, int millis) {}

    default void rumbleBlips(int count) {}

    default void led(double red, double green, double blue) {}
}
