package org.firstinspires.ftc.teamcode.Autonomous.Utils;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class MotionProfiling2d {

    private double maxAccel, maxVel;

    private Vector2d xt, vt;

    private Vector2d x0, x1, x2;
    private Vector2d v0, v1, v2;
    private Vector2d a0, a1, a2;

    private double t1, t2, t3;

    public MotionProfiling2d(double maxAccel, double maxVel) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
    }

    public void setTarget(Vector2d xt, Vector2d vt, Vector2d x0, Vector2d v0) {
        this.xt = xt;
        this.vt = vt.norm() > maxVel ? vt.times(maxVel / vt.norm()) : vt;

        Vector2d dx = xt.minus(x0);
        if(xt.epsilonEquals(x0)) {
            this.x0 = this.x1 = this.x2 = x0;
            this.v0 = this.v1 = this.v2 = v0;
            this.a0 = this.a1 = this.a2 = new Vector2d(0.0, 0.0);
            this.t1 = this.t2 = this.t3 = 0.0;
            return;
        }

        Vector2d dir = dx.div(dx.norm());
        double delta = dx.norm() * maxAccel + 0.5 * (vt.dot(vt) + v0.dot(v0));

        Vector2d vm = dir.times(maxVel);

        double step1 = vm.minus(v0).norm() / maxAccel;
        double step2 = delta / (maxVel * maxAccel) - maxVel / maxAccel;
        double step3 = vm.minus(vt).norm() / maxAccel;

        t1 = step1;
        t2 = t1 + step2;
        t3 = t2 + step3;

        if(t2 <= t1) {
            Vector2d v = dir.times(Math.sqrt(delta));
            t1 = t2 = v.minus(v0).norm() / maxAccel;
            t3 = t1 + v.minus(vt).norm() / maxAccel;
        }

        this.a0 = dir.times(maxAccel);
        this.v0 = v0;
        this.x0 = x0;

        this.a1 = new Vector2d(0.0, 0.0);
        this.v1 = this.v0.plus(this.a0.times(t1));
        this.x1 = this.x0.plus(this.v0.times(t1)).plus(this.a0.times(0.5 * t1 * t1));

        this.a2 = dir.times(-maxAccel);
        this.v2 = this.v1.plus(this.a1.times(t2 - t1));
        this.x2 = this.x1.plus(this.v1.times(t2 - t1)).plus(this.a1.times(0.5 * (t2 - t1) * (t2 - t1)));
    }

    public void setConstraints(double maxAccel, double maxVel) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
    }

    public State2d getState(double t) {

        if(t < 0.0)
            return new State2d(x0, v0, new Vector2d(0.0, 0.0));

        if(t3 <= t)
            return new State2d(xt, vt, new Vector2d(0.0, 0.0));

        double dt;
        Vector2d x0, v0, a0;

        if(t < t1) {
            a0 = this.a0;
            v0 = this.v0;
            x0 = this.x0;
            dt = t;
        } else if(t < t2) {
            a0 = this.a1;
            v0 = this.v1;
            x0 = this.x1;
            dt = t - t1;
        } else {
            a0 = this.a2;
            v0 = this.v2;
            x0 = this.x2;
            dt = t - t2;
        }

        Vector2d accel = a0;
        Vector2d vel   = v0.plus(a0.times(dt));
        Vector2d pos   = x0.plus(v0.times(dt)).plus(a0.times(0.5 * dt * dt));

        return new State2d(pos, vel, accel);
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

    public static class State2d {

        public final Vector2d position;
        public final Vector2d velocity;
        public final Vector2d acceleration;

        public State2d(final Vector2d position, final Vector2d velocity, final Vector2d acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }

    }

}
