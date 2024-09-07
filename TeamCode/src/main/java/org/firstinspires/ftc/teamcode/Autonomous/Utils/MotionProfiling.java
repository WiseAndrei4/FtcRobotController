package org.firstinspires.ftc.teamcode.Autonomous.Utils;

import com.qualcomm.robotcore.util.Range;

public class MotionProfiling {

    private double maxAccel, maxVel;

    private double xt, vt;

    private double x0, x1, x2;
    private double v0, v1, v2;
    private double a0, a1, a2;

    private double t1, t2, t3;

    public MotionProfiling(double maxAccel, double maxVel) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
    }

    public void setTarget(double xt, double vt, double x0, double v0) {
        this.xt = xt;
        this.vt = Range.clip(vt, -maxVel, maxVel);

        double dx = xt - x0;
        double sigma = Math.signum(dx);
        double delta = Math.abs(dx) * maxAccel + 0.5 * (vt * vt + v0 * v0);

        double step1 = (maxVel - sigma * v0) / maxAccel;
        double step2 = delta / (maxVel * maxAccel) - maxVel / maxAccel;
        double step3 = (maxVel - sigma * vt) / maxAccel;

        t1 = step1;
        t2 = t1 + step2;
        t3 = t2 + step3;

        if(t2 <= t1) {
            double v = sigma * Math.sqrt(delta);
            t1 = t2 = sigma * (v - v0) / maxAccel;
            t3 = t1 + sigma * (v - vt) / maxAccel;
        }

        this.a0 = sigma * maxAccel;
        this.v0 = v0;
        this.x0 = x0;

        this.a1 = 0.0;
        this.v1 = this.v0 + this.a0 * t1;
        this.x1 = this.x0 + this.v0 * t1 + 0.5 * this.a0 * t1 * t1;

        this.a2 = -sigma * maxAccel;
        this.v2 = this.v1 + this.a1 * (t2 - t1);
        this.x2 = this.x1 + this.v1 * (t2 - t1) + 0.5 * this.a1 * (t2 - t1) * (t2 - t1);
    }

    public void setConstraints(double maxAccel, double maxVel) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
    }

    public State getState(double t) {

        if (t < 0.0)
            return new State(x0, v0, 0.0);

        if (t3 <= t)
            return new State(xt, vt, 0.0);

        double dt, x0, v0, a0;

        if (t < t1) {
            x0 = this.x0;
            v0 = this.v0;
            a0 = this.a0;
            dt = t;
        } else if (t < t2) {
            x0 = this.x1;
            v0 = this.v1;
            a0 = this.a1;
            dt = (t - t1);
        } else {
            x0 = this.x2;
            v0 = this.v2;
            a0 = this.a2;
            dt = (t - t2);
        }

        double a = a0;
        double v = v0 + a0 * dt;
        double x = x0 + v0 * dt + 0.5 * a0 * dt * dt;

        return new State(x, v, a);
    }

    public double getCruiseTime() {
        return t1;
    }

    public double getDecelerateTime() {
        return t2;
    }

    public double getFinishTime() {
        return t3;
    }

    public static class State {
        public final double position;
        public final double velocity;
        public final double acceleration;

        public State(final double position, final double velocity, final double acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
    }

}
