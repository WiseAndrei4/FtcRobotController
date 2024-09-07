package org.firstinspires.ftc.teamcode.Algo;

public class AnalogInputState {

    private final double deadzone;

    private double value = 0.0;

    public AnalogInputState(final double deadzone) {
        this.deadzone = deadzone;
    }

    public void update(final double value) {
        this.value = (Math.abs(value) <= deadzone ? 0.0 : value);
    }

    public double get() {
        return this.value;
    }

}
