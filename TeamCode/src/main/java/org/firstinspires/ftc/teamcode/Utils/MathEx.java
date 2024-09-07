package org.firstinspires.ftc.teamcode.Utils;

public class MathEx {

    public static double Lerp(double t, double min, double max) {
        return min * (1.0 - t) + max * t;
    }

    public static float Lerp(float t, float min, float max) {
        return min * (1.0f - t) + max * t;
    }

    public static int Lerp(double t, int min, int max) {
        return (int) (min * (1.0 - t) + max * t);
    }

}
