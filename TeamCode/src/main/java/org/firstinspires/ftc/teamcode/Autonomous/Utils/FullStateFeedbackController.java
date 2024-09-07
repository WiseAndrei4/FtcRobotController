package org.firstinspires.ftc.teamcode.Autonomous.Utils;

public class FullStateFeedbackController {

    private double k1;
    private double k2;

    public FullStateFeedbackController(double k1, double k2) {
        this.k1 = k1;
        this.k2 = k2;
    }

    public double calculate(double positionError, double velocityError) {
        return k1 * positionError + k2 * velocityError;
    }

    public void setCoefficients(double k1, double k2) {
        this.k1 = k1;
        this.k2 = k2;
    }

}
