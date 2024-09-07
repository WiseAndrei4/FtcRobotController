package org.firstinspires.ftc.teamcode;

public class Configuration {

    public static final String CAMERA = "Webcam 1";

    public class LIFT {
        public static final String LEFT_LIFT = "lift1";
        public static final String RIGHT_LIFT = "lift2";
    }

    public class ClawConfiguration {
        public static final String LEFT = "leftClaw"; // port 0
        public static final String RIGHT = "rightClaw"; // port 1
    }

    public class TractionConfiguration {
        public static final String FRONT_LEFT = "frontLeft";
        public static final String FRONT_RIGHT = "frontRight";
        public static final String BACK_LEFT = "backLeft";
        public static final String BACK_RIGHT = "backRight";

        public static final String LEFT_ENCODER = "leftEncoder";
        public static final String RIGHT_ENCODER = "rightEncoder";
        public static final String FRONT_ENCODER = BACK_LEFT;

    }
    public class ArmConfiguration{
        public static final String LEFT="leftArm";
        public static final String RIGHT="rightArm";
    }
    public class ExtenderArmConfiguration {
        public static final String LEFT = "extendLeft";
        public static final String RIGHT = "extendRight";
    }
    public class SensorConfiguration {
        public static final String SENSOR_IR_MAIN = "sensorIRMain";
        public static final String SENSOR_IR_LEFT = "sensorIRLeft";
        public static final String SENSOR_IR_RIGHT = "sensorIRRight";
        public static final String SENSOR_DISTNACE = "distance";

        public static final String SENSOR_COLOR_LEFT = "sensorColorLeft";
        public static final String SENSOR_COLOR_RIGHT = "sensorColorRight";
    }
    public class SensorDistanceConfiguration{
        public static final String SENSOR_DISTANCE1 = "distance1";
        public static final String SENSOR_DISTANCE2 = "distance2";
    }

    public class AirplaneLauncherConfiguration{
        public static final String LAUNCHER="launcherServo";
    }
    public class ArmRotationConfiguration{
        public static final String ARM_ROTATION="rotationServo";
    }
    public class StackIntakeConfiguration{
        public static final String LEFT="intakeLeft";
        public static final String RIGHT="intakeRight";
    }
    public class MotorIntakeConfiguration{
        public static final String INTAKE = "motorIntake";
    }
    public class OuttakeSingleExtenderConfiguration{
        public static final String SERVO_EXTENDER_ARM="servoArmExtender";

    }
    public class OuttakeExtenderConfiguration{
        public static final String LEFT="extendedLeftOuttake";
        public static final String RIGHT = "extendedRightOuttake";

    }
    public class IntakeExtenderConfiguration{
        public static final String SERVO_EXTENDER = "servoIntake";
    }
}