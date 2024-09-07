package org.firstinspires.ftc.teamcode.Algo.Time;

public class Time {

    private static double currentTime = System.nanoTime() * 1e-9;
    private static double dt = 0.0;
    private static double startTime = 0.0, sinceStartTime = 0.0;

    public static double time() {
        return currentTime;
    }

    public static double deltaTime() {
        return dt;
    }

    public static double sinceStart() {
        return sinceStartTime;
    }

    public static void start() {
        startTime = System.nanoTime() * 1e-9;
    }

    public static void update() {
        double time = System.nanoTime() * 1e-9;
        dt = time - currentTime;
        currentTime = time;

        sinceStartTime = time - startTime;
    }

}
