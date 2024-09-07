package org.firstinspires.ftc.teamcode.Algo.Time;

public class Timer {

    private double startTime;
    private double waitTime;

    public Timer(final double waitTime) {
        this.startTime = Time.time();
        this.waitTime = waitTime;
    }
    public void reset() {
        this.startTime = Time.time();
    }

    public void reset(final double waitTime) {
        this.startTime = Time.time();
        this.waitTime = waitTime;
    }

    public boolean isDone() {
        return Time.time() >= this.startTime + this.waitTime;
    }

    public double getStartTime() {
        return startTime;
    }

    public double getWaitTime() {
        return waitTime;
    }

    public double getTimeLeft() {
        return Time.time() - (this.startTime + this.waitTime);
    }

}

