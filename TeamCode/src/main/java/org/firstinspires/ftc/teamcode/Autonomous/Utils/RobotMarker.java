package org.firstinspires.ftc.teamcode.Autonomous.Utils;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

import org.firstinspires.ftc.teamcode.Controllers.ControllerManager;
import org.firstinspires.ftc.teamcode.Controllers.DualArmController;
import org.firstinspires.ftc.teamcode.Controllers.DualExtenderArmController;
import org.firstinspires.ftc.teamcode.Controllers.DualLiftController;
import org.firstinspires.ftc.teamcode.Controllers.IndividualControllers.ArmController;
import org.firstinspires.ftc.teamcode.Controllers.IndividualControllers.ArmRotationController;
import org.firstinspires.ftc.teamcode.Controllers.IndividualControllers.ClawController;
import org.firstinspires.ftc.teamcode.Controllers.IndividualControllers.LiftController;
import org.firstinspires.ftc.teamcode.Controllers.StackIntakeController;

public class RobotMarker implements MarkerCallback {

    private DualArmController armController;
    private DualLiftController liftController;
    private ClawController clawControllerLeft;
    private ClawController clawControllerRight;
    private StackIntakeController stackIntakeController;

    private DualExtenderArmController extenderController;
    private ArmRotationController rotationController;

    private ArmRotationController.POSITIONS armRotation = null;
    private ClawController.POSITIONS clawLeftPosition = null;
    private ClawController.POSITIONS clawRightPosition = null;
    private StackIntakeController.POSITIONS intakeRightPosition = null;
    private StackIntakeController.POSITIONS intakeLeftPosition = null;
    private DualExtenderArmController.POSITIONS extenderPosition = null;
    private ArmController.POSITIONS armPosition = null;//ArmController.POSITIONS.PICKUP_PIXEL_POSITION;
    private LiftController.Positions liftPosition = null;//DualLiftController.POSITIONS.PICKUP;

    public RobotMarker(ControllerManager controllerManager) {
        armController = controllerManager.get(DualArmController.class, "dualArm");
        liftController = controllerManager.get(DualLiftController.class, "lift");
        clawControllerLeft = controllerManager.get(ClawController.class, "clawLeft");
        clawControllerRight = controllerManager.get(ClawController.class, "clawRight");
        stackIntakeController = controllerManager.get(StackIntakeController.class, "stackIntake");
        extenderController = controllerManager.get(DualExtenderArmController.class, "extender");
        rotationController = controllerManager.get(ArmRotationController.class, "armRotation");
    }

    public RobotMarker arm(ArmController.POSITIONS position) {
        armPosition = position;
        return this;
    }

    public RobotMarker lift(LiftController.Positions position) {
        liftPosition = position;
        return this;
    }

    public RobotMarker claw(ClawController.POSITIONS position, ArmController.ARM_ID arm_id) {
        switch (arm_id) {
            case LEFT: {
                clawLeftPosition = position;
                break;
            }
            case RIGHT: {
                clawRightPosition = position;
                break;
            }
        }
        return this;
    }

    public RobotMarker intake(StackIntakeController.POSITIONS position, StackIntakeController.IntakeID id) {
        switch (id) {
            case LEFT: {
                intakeLeftPosition = position;
                break;
            }
            case RIGHT: {
                intakeRightPosition = position;
                break;
            }
        }
        return this;
    }

    public RobotMarker intake(StackIntakeController.POSITIONS position) {
        intakeLeftPosition = position;
        intakeRightPosition = position;
        return this;
    }

    public RobotMarker extenders(DualExtenderArmController.POSITIONS position) {
        extenderPosition = position;
        return this;
    }

    public RobotMarker armRotation(ArmRotationController.POSITIONS position) {
        armRotation = position;
        return this;
    }

    @Override
    public void onMarkerReached() {
        if(armController != null && armPosition != null) {
            armController.goTo(armPosition);
        }

        if (liftController != null && liftPosition != null) {
            liftController.goTo(liftPosition);
        }

        if (clawControllerLeft != null && clawLeftPosition != null) {
            clawControllerLeft.goTo(clawLeftPosition);
        }

        if (clawControllerRight != null && clawRightPosition != null) {
            clawControllerRight.goTo(clawRightPosition);
        }

        if (stackIntakeController != null && intakeLeftPosition != null)
            stackIntakeController.goTo(intakeLeftPosition, StackIntakeController.IntakeID.LEFT);

        if (stackIntakeController != null && intakeRightPosition != null)
            stackIntakeController.goTo(intakeRightPosition, StackIntakeController.IntakeID.RIGHT);

        if (extenderController != null && extenderPosition != null)
            extenderController.goTo(extenderPosition);

        if (rotationController != null && armRotation != null)
            rotationController.goTo(armRotation);
    }

}