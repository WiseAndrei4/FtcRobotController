package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Algo.Time.Time;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Configuration;

/**
 * A class for controlling mecanum traction.
 */
public class TractionController extends Controller {

    // Motors for the traction
    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;

    private StandardTrackingWheelLocalizer localizer;
    //private BHI260IMU imu;

    private final MotorPowerProfile powerProfile = new MotorPowerProfile();
    private final PIDController headingController = new PIDController(0.01, 0.0, 0.0, 0.1);

    private double headingOffset;

    // The scale of the motors' powers
    private float powerScale;

    private double headingTarget = 0.0;
    private boolean keepHeading = false;
    private boolean fieldCentric = false;
    private boolean frana = false;

    /*private BHI260IMU initIMU(final HardwareMap hardwareMap, final String name) {
        BHI260IMU imu = hardwareMap.get(BHI260IMU.class, name);
        imu.initialize();
        return imu;
    }*/

    /**
     * Gets a motor from the hardware map given.
     * Behaviour undefined if there is no such motor in the hardware map.
     *
     * @param hardwareMap the hardware map to get the motor from
     * @param name        the name of the motor to get
     * @return the motor requested
     */
    public DcMotorEx initMotor(HardwareMap hardwareMap, String name) {
        // get the motor
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }


    /**
     * Initializes the traction controller from the given hardware map0
     *
     * @param hardwareMap the hardware map to get the motors from
     */
    public TractionController(HardwareMap hardwareMap, float powerScale) {
        // get all motors
        leftFront = initMotor(hardwareMap, Configuration.TractionConfiguration.FRONT_LEFT);
        rightFront = initMotor(hardwareMap, Configuration.TractionConfiguration.FRONT_RIGHT);
        leftRear = initMotor(hardwareMap, Configuration.TractionConfiguration.BACK_LEFT);
        rightRear = initMotor(hardwareMap, Configuration.TractionConfiguration.BACK_RIGHT);

        // set the directions of the motors; the right are mirrored from
        // the left, so the direction must be reversed
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        //localizer=new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>());

        //imu = initIMU(hardwareMap, "imu");
        headingOffset = getRawHeading();

        // set the scale of the motors' powers
        this.powerScale = powerScale;
    }

    public boolean isCalibrated() {
        return true;//return imu.isSystemCalibrated() && imu.isGyroCalibrated() && imu.isAccelerometerCalibrated() && imu.isMagnetometerCalibrated();
    }

    public void resetDefaultAngle() {
        headingOffset = getRawHeading();
    }

    public void setPowerScale(float powerScale) {
        this.powerScale = powerScale;
    }

    /**
     * Updates the motor powers for mecanum drive with some inputs.
     *
     * @param forward  the forward input. Must be in [-1, 1]
     * @param strafe   the strafe input. Must be in [-1, 1]
     * @param rotation the rotation input. Must be in [-1, 1]
     */
    public void update(double forward, double strafe, double rotation, Telemetry telemetry) {
        // Calculate field centric controls if field centric drive is enabled
        if (fieldCentric) {
            double heading = Math.toRadians(getRobotHeading());
            double fieldCentricForward = forward * Math.cos(heading) + strafe * Math.sin(heading);
            double fieldCentricStrafe = strafe * Math.cos(heading) - forward * Math.sin(heading);


            forward = fieldCentricForward;
            strafe = fieldCentricStrafe;
        }

        telemetry.addData("Chassis Rotation", rotation);

        // Reset the power profile and apply the drive controls
        powerProfile.reset();


        applyDrive(forward * powerScale, strafe * powerScale, powerProfile);

        // Apply heading correction if heading is automatically controlled, or manual rotation otherwise
        if (keepHeading) {
            applyHeadingCorrection(powerProfile);
        } else applyRotation(rotation * powerScale, powerProfile);

        // Make sure nothing exceeds powerScale, while proportions remain the same
        powerProfile.uniformClamp(powerScale);

        // set the powers of the motors
        leftFront.setPower(powerProfile.leftFront);
        rightFront.setPower(powerProfile.rightFront);
        leftRear.setPower(powerProfile.leftRear);
        rightRear.setPower(powerProfile.rightRear);

        telemetry.addData("Heading", getRobotHeading());
        telemetry.addData("Left Front Current", leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Front Current", rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Left Rear Current", leftRear.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Rear Current", rightRear.getCurrent(CurrentUnit.AMPS));

        //telemetry.addData("Heading error", getHeadingError());

        ///telemetry.addData("Motor VOLTAGE USED: ", (leftFront.getPower()+rightFront.getPower()+leftRear.getPower()+rightRear.getPower())*12);
        /// ??
        //telemetry.addData("IMU calibrated", imu.getCalibrationStatus());
    }

    public void applyDrive(final double forward, final double strafe, final MotorPowerProfile powerProfile) {
        // adjust the powers for motor power
        powerProfile.leftFront += forward + strafe;
        powerProfile.rightFront += forward - strafe;
        powerProfile.leftRear += forward - strafe;
        powerProfile.rightRear += forward + strafe;
    }

    public void applyRotation(final double rotation, final MotorPowerProfile powerProfile) {
        powerProfile.leftFront += -rotation;
        powerProfile.rightFront += rotation;
        powerProfile.leftRear += -rotation;
        powerProfile.rightRear += rotation;
    }

    public void applyHeadingCorrection(MotorPowerProfile powerProfile) {
        double headingError = getHeadingError();

        if (Math.abs(headingError) <= 0.1)
            return;

        double coef = headingController.step(headingError, Time.deltaTime());
        coef = Range.clip(coef, -powerScale, powerScale);

        applyRotation(-coef, powerProfile);
    }

    public void setHeadingTarget(double headingTarget) {
        this.headingTarget = headingTarget;
    }

    public void setKeepHeading(boolean keepHeading) {
        this.keepHeading = keepHeading;
        if (this.keepHeading)
            headingController.reset();
    }

    public void toggleKeepHeading() {
        setKeepHeading(!keepHeading);
    }

    public void setFieldCentric(final boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public void toggleFieldCentric() {
        setFieldCentric(!fieldCentric);
    }

    public double getHeadingError() {
        double headingError = headingTarget - getRobotHeading();
        while (headingError > 180.0) headingError -= 360.0;
        while (headingError <= -180.0) headingError += 360.0;
        return headingError;
    }

    public double getRobotHeading() {
        return getRawHeading() - headingOffset;
    }

    public double getRawHeading() {
        return headingTarget;//return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public enum MotorPosition {
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
    }

    public int getEncoderPosition(MotorPosition position) {
        if (position == MotorPosition.FRONT_LEFT)
            return leftFront.getCurrentPosition();
        else if (position == MotorPosition.FRONT_RIGHT)
            return rightFront.getCurrentPosition();
        else if (position == MotorPosition.BACK_LEFT)
            return leftRear.getCurrentPosition();
        return rightRear.getCurrentPosition();
    }

    public static class MotorPowerProfile {
        public double leftFront = 0.0;
        public double rightFront = 0.0;
        public double leftRear = 0.0;
        public double rightRear = 0.0;

        public void uniformClamp(final double bound) {
            final double max = Math.max(Math.max(leftFront, rightFront), Math.max(leftRear, rightRear));
            if (bound < max) {
                leftFront *= bound / max;
                rightFront *= bound / max;
                leftRear *= bound / max;
                rightRear *= bound / max;
            }
        }

        public void reset() {
            leftFront = 0.0;
            rightFront = 0.0;
            leftRear = 0.0;
            rightRear = 0.0;
        }
    }

    public static class PIDController {

        public final double P;
        public final double I;
        public final double D;
        public final double integral_threshold;

        private double integral = 0.0;
        private double lastError = 0.0;

        public PIDController(final double P, final double I, final double D, final double integral_threshold) {
            this.P = P;
            this.I = I;
            this.D = D;
            this.integral_threshold = integral_threshold;
        }

        public double step(double error, double deltaTime) {
            double proportional, derivative;

            proportional = error;
            integral = Range.clip(integral + error * deltaTime, -integral_threshold, integral_threshold);
            derivative = Range.clip(deltaTime != 0.0 ? (error - lastError) / deltaTime : 0.0, -1.0, 1.0);

            lastError = error;

            return proportional * P + integral * I + derivative * D;
        }

        public void reset() {
            integral = 0.0;
            lastError = 0.0;
        }

    }
}