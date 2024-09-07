package org.firstinspires.ftc.teamcode.Autonomous.Utils;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.Controllers.ControllerManager;
import org.firstinspires.ftc.teamcode.Controllers.DualLiftController;
import org.firstinspires.ftc.teamcode.Controllers.IndividualControllers.ClawController;
import org.firstinspires.ftc.teamcode.Controllers.IndividualControllers.DistanceSensorsController;
import org.firstinspires.ftc.teamcode.Controllers.IndividualControllers.IntakeMotorController;
import org.firstinspires.ftc.teamcode.Controllers.IndividualControllers.IntakeServoExtenderArmController;
import org.firstinspires.ftc.teamcode.Controllers.IndividualControllers.LiftController;
import org.firstinspires.ftc.teamcode.Controllers.IndividualControllers.OuttakeRotationAngleArmController;
import org.firstinspires.ftc.teamcode.Controllers.IndividualControllers.OuttakeSingleServoExtenderController;
import org.firstinspires.ftc.teamcode.Controllers.OuttakeDualServoExtenderController;

public class NewRobotMarker implements MarkerCallback {

    private double motorPower = 0.8;
    private DualLiftController dualLiftController;
    private ClawController clawControllerLeft;
    private ClawController clawControllerRight;

    private IntakeMotorController intakeMotorController;
    private IntakeServoExtenderArmController intakeExtenderArmController;

    private OuttakeDualServoExtenderController outtakeDualServoExtenderController;
    private OuttakeSingleServoExtenderController outtakeSingleExtenderArmController;

    private OuttakeRotationAngleArmController armRotationController;

    private DistanceSensorsController sensorDistanceFirstController;
    private DistanceSensorsController sensorDistanceSecondController;




    private IntakeMotorController.DIRECTIONS motorDirection = null;
    private IntakeServoExtenderArmController.POSITIONS intakeExtenderPosition = null;

    private OuttakeSingleServoExtenderController.POSITIONS extenderPosition = null;

    private OuttakeDualServoExtenderController.POSITIONS armExtenderPosition = null;

    private OuttakeRotationAngleArmController.POSITIONS armRotation = null;
    private ClawController.POSITIONS clawLeftPosition = null;
    private ClawController.POSITIONS clawRightPosition = null;

    public static Boolean sensor1State = null;
    public static Boolean sensor2State = null;

    private LiftController.Positions liftPosition = null;

    public NewRobotMarker(ControllerManager controllerManager) {

        dualLiftController = controllerManager.get(DualLiftController.class, "lift");

        clawControllerLeft = controllerManager.get(ClawController.class, "clawLeft");
        clawControllerRight = controllerManager.get(ClawController.class, "clawRight");



         sensorDistanceFirstController = controllerManager.get(DistanceSensorsController.class, "distance1");
         sensorDistanceSecondController = controllerManager.get(DistanceSensorsController.class, "distance2");

        intakeMotorController = controllerManager.get(IntakeMotorController.class, "motorIntake");
        intakeExtenderArmController = controllerManager.get(IntakeServoExtenderArmController.class, "intakeExtender");

        outtakeDualServoExtenderController = controllerManager.get(OuttakeDualServoExtenderController.class, "longExtender");
        outtakeSingleExtenderArmController = controllerManager.get(OuttakeSingleServoExtenderController.class, "shortExtender");

        armRotationController = controllerManager.get(OuttakeRotationAngleArmController.class, "armRotation");


    }

    public NewRobotMarker rotation(OuttakeRotationAngleArmController.POSITIONS position) {
        armRotation = position;
        return this;
    }

    public NewRobotMarker lift(LiftController.Positions position) {
        liftPosition = position;
        return this;
    }

    public NewRobotMarker intakeExtender(IntakeServoExtenderArmController.POSITIONS position) {
        intakeExtenderPosition = position;
        return this;
    }

    public NewRobotMarker extender(OuttakeSingleServoExtenderController.POSITIONS position) {
        extenderPosition = position;
        return this;
    }

    public NewRobotMarker armExtender(OuttakeDualServoExtenderController.POSITIONS position) {
        armExtenderPosition = position;
        return this;
    }

    public NewRobotMarker claw(ClawController.POSITIONS position, String name) {
        switch (name) {
            case Configuration.ClawConfiguration.LEFT: {
                clawLeftPosition = position;
                break;
            }
            case Configuration.ClawConfiguration.RIGHT: {
                clawRightPosition = position;
                break;
            }
        }
        return this;
    }

    /*public NewRobotMarker detection(String name, boolean state){
        switch (name) {
            case Configuration.SensorDistanceConfiguration.SENSOR_DISTANCE1: {
                sensor1State = sensorDistanceFirstController.detected();
                break;
            }
            case Configuration.SensorDistanceConfiguration.SENSOR_DISTANCE2: {
                sensor2State = sensorDistanceSecondController.detected();
                break;
            }
        }
        return this;
    }*/
    public NewRobotMarker motor(double motorPower, IntakeMotorController.DIRECTIONS direction) {
        motorDirection = direction;
        this.motorPower = motorPower;
        intakeMotorController.run(motorPower, direction);
        return this;
    }

    @Override
    public void onMarkerReached() {
        if (armRotationController != null && armRotation != null) {
            armRotationController.goTo(armRotation);
        }

        if (dualLiftController != null && liftPosition != null) {
            dualLiftController.goTo(liftPosition);
        }

        if (clawControllerLeft != null && clawLeftPosition != null) {
            clawControllerLeft.goTo(clawLeftPosition);
        }

        if (clawControllerRight != null && clawRightPosition != null) {
            clawControllerRight.goTo(clawRightPosition);
        }

        if (outtakeDualServoExtenderController != null && armExtenderPosition != null) {
            outtakeDualServoExtenderController.goTo(armExtenderPosition);
        }
        if(outtakeSingleExtenderArmController != null && extenderPosition != null){
            outtakeSingleExtenderArmController.goTo(extenderPosition);
        }
        if(intakeExtenderArmController != null && intakeExtenderPosition!=null){
            intakeExtenderArmController.goTo(intakeExtenderPosition);
        }
        if(intakeMotorController != null && motorDirection != null ){
            intakeMotorController.run(motorPower, motorDirection);
        }
    /*    if(sensor1State != null &&  sensorDistanceFirstController!=null){
            sensor1State = sensorDistanceFirstController.detected();
        }
       if(sensor1State != null &&  sensorDistanceSecondController!=null){
            sensor2State = sensorDistanceFirstController.detected();
        }*/

    }
}
