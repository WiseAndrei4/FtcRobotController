package org.firstinspires.ftc.teamcode.Autonomous.Utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MPConstants {

    public static double LATERAL_MULTIPLIER = 1.264450158291902;

    public static double MAX_VEL = 50;
    public static double MAX_ACCEL = 50;

    public static double MAX_ANG_VEL = Math.toRadians(300);
    public static double MAX_ANG_ACCEL = Math.toRadians(300);

    public static AdmissibleError ADMISSIBLE_ERROR = new AdmissibleError(0.1, Math.toRadians(0.5));

    public static FeedbackConstants FORWARD_FEEDBACK = new FeedbackConstants(0.005, 0.0003);
    public static FeedbackConstants STRAFE_FEEDBACK = new FeedbackConstants(0.03, 0.01);
    public static FeedforwardConstants FORWARD_FEEDFORWARD = new FeedforwardConstants(0.011, 0.0045);
    public static FeedforwardConstants STRAFE_FEEDFORWARD = new FeedforwardConstants(0.020, 0.006);

    public static FeedbackConstants HEADING_FEEDBACK = new FeedbackConstants(2.5, 0.1888);
    public static FeedforwardConstants HEADING_FEEDFORWARD = new FeedforwardConstants(0.06, 0.005);

    public static class FeedbackConstants {

        public double K1;
        public double K2;

        public FeedbackConstants() {
            this(0.0, 0.0);
        }

        public FeedbackConstants(double K1, double K2) {
            this.K1 = K1;
            this.K2 = K2;
        }
    }

    public static class FeedforwardConstants {

        public double KV;
        public double KA;

        public FeedforwardConstants() {
            this(0.0, 0.0);
        }

        public FeedforwardConstants(double KV, double KA) {
            this.KV = KV;
            this.KA = KA;
        }

    }

    public static class AdmissibleError {

        public double TRANSLATION;
        public double HEADING;

        public AdmissibleError() {
            this(0.5, Math.toRadians(0.5));
        }

        public AdmissibleError(double translation, double heading) {
            TRANSLATION = translation;
            HEADING = heading;
        }
    }

}
