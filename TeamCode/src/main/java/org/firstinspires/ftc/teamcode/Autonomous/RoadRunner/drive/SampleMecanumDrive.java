package org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.Configuration;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
  /*  public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8.5, 0, 2.5);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(28.053, 0, 1.288);*/
  public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(7.5, 1, 1);


    public static double LATERAL_MULTIPLIER = 1.705;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static double MANUAL_KP = 1.0;
    public static double MANUAL_FF = 4.0;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(DriveConstants.MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    private boolean manualControl = false;

    private Pose2d targetPose = new Pose2d(0.0, 0.0, 0.0);

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, DriveConstants.TRACK_WIDTH, DriveConstants.TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(0.5)), 1.0);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        /*imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);*/
        /*imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);*/

       /* leftFront = hardwareMap.get(DcMotorEx.class, Configuration.TractionConfiguration.BACK_RIGHT);
        leftRear = hardwareMap.get(DcMotorEx.class, Configuration.TractionConfiguration.FRONT_RIGHT);
        rightRear = hardwareMap.get(DcMotorEx.class, Configuration.TractionConfiguration.FRONT_LEFT);
        rightFront = hardwareMap.get(DcMotorEx.class, Configuration.TractionConfiguration.BACK_LEFT);*/

        leftFront = hardwareMap.get(DcMotorEx.class, Configuration.TractionConfiguration.BACK_LEFT);
        leftRear = hardwareMap.get(DcMotorEx.class, Configuration.TractionConfiguration.FRONT_LEFT);
        rightRear = hardwareMap.get(DcMotorEx.class, Configuration.TractionConfiguration.FRONT_RIGHT);
        rightFront = hardwareMap.get(DcMotorEx.class, Configuration.TractionConfiguration.BACK_RIGHT);

        /*leftFront = hardwareMap.get(DcMotorEx.class, Configuration.TractionConfiguration.FRONT_LEFT);
        rightRear = hardwareMap.get(DcMotorEx.class, Configuration.TractionConfiguration.BACK_RIGHT);
        leftRear = hardwareMap.get(DcMotorEx.class, Configuration.TractionConfiguration.BACK_LEFT);
        rightFront = hardwareMap.get(DcMotorEx.class, Configuration.TractionConfiguration.FRONT_RIGHT);*/


      /*  leftFront = hardwareMap.get(DcMotorEx.class, Configuration.TractionConfiguration.FRONT_RIGHT);
        leftRear = hardwareMap.get(DcMotorEx.class, Configuration.TractionConfiguration.BACK_RIGHT);
        rightFront = hardwareMap.get(DcMotorEx.class, Configuration.TractionConfiguration.FRONT_LEFT);
        rightRear = hardwareMap.get(DcMotorEx.class, Configuration.TractionConfiguration.BACK_LEFT);*/



        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (DriveConstants.RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (DriveConstants.RUN_USING_ENCODER && DriveConstants.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

       /* rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);*/

        /*leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);*/

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // TODO: if desired, use setLocalizer() to change the localization method
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));
       // setLocalizer(new TwoWheelTrackingLocalizer(hardwareMap, this));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void updateManual(Telemetry telemetry) {
        Pose2d estimate = getPoseEstimate();
        double diffx;
        double diffy;
        double diffh;

        diffx = targetPose.getX() - estimate.getX();
        diffy = targetPose.getY() - estimate.getY();
        diffh = targetPose.getHeading() - estimate.getHeading();

        if(diffh >= Math.toRadians(180.0))
            diffh -= Math.toRadians(360.0);

        if(diffh <= Math.toRadians(-180.0))
            diffh += Math.toRadians(360.0);

        double vx = MANUAL_KP * diffx + MANUAL_FF * Math.signum(diffx);
        if (Math.abs(diffx) <= 0.1)
            vx = 0.0;

        double vy = MANUAL_KP * diffy + MANUAL_FF * Math.signum(diffy);
        if (Math.abs(diffy) <= 0.1)
            vy = 0.0;

        double vh = 2.0 * diffh + 1.5 * Math.signum(diffh);
        if(Math.abs(diffh) <= Math.toRadians(1.0))
            vh = 0.0;

        double t = estimate.getHeading();
        double vxr = vy * Math.sin(t) + vx * Math.cos(t);
        double vyr = 1.8 * (-vx * Math.sin(t) + vy * Math.cos(t));
        double vhr = vh;

        DriveSignal driveSignal = new DriveSignal(new Pose2d(vxr, vyr, vhr), new Pose2d(0.0, 0.0, 0.0));
        setDriveSignal(driveSignal);

        if (telemetry != null) {
            telemetry.addData("Drive Error X", diffx);
            telemetry.addData("Drive Error Y", diffy);
            telemetry.addData("Heading Error", diffh);
        }
    }

    public void updateAuto(Telemetry telemetry) {
        targetPose = getPoseEstimate();

        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void update() {
        update(null);
    }

    public void update(Telemetry telemetry) {
        updatePoseEstimate();

        if(manualControl)
            updateManual(telemetry);
        else updateAuto(telemetry);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy() || manualControl;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(DriveConstants.encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(DriveConstants.encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0.0; //imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return 0.0;//(double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void setManualControl(boolean manualControl) {
        this.manualControl = manualControl;
    }

    public boolean getManualControl() {
        return this.manualControl;
    }

    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
    }

    public Pose2d getTargetPose() {
        return this.targetPose;
    }

    public boolean isOnTarget() {
        Pose2d estimate = getPoseEstimate();

        double diffx = targetPose.getX() - estimate.getX();
        double diffy = targetPose.getY() - estimate.getY();

        return Math.abs(diffx) <= 0.1 && Math.abs(diffy) <= 0.1;
    }
}
