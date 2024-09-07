package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Algo.AnalogInputState;
import org.firstinspires.ftc.teamcode.Algo.Time.Time;
import org.firstinspires.ftc.teamcode.Controllers.ControllerManager;
import org.firstinspires.ftc.teamcode.Controllers.TractionController;


@Disabled

@TeleOp(name = "TractionDebugger",group = "TeleOp")
public class TractionDebugger extends LinearOpMode {

    private final AnalogInputState leftStickX = new AnalogInputState(0.1);
    private final AnalogInputState leftStickY = new AnalogInputState(0.1);
    private final AnalogInputState rightStickX = new AnalogInputState(0.1);

    private final float MAX_TRACTION_POWER = 1.0f;
    private final float powerPercentage = 1.0f;
    @Override
    public void runOpMode() throws InterruptedException {
        ControllerManager controllerManager = new ControllerManager();
        controllerManager.addController("traction", new TractionController(hardwareMap, 0.7f));

        TractionController tractionController = controllerManager.get(TractionController.class, "traction");

        tractionController.setPowerScale(powerPercentage * MAX_TRACTION_POWER);

        telemetry.addLine("IMU is calibrating...");

        telemetry.update();
        while (opModeIsActive() && !tractionController.isCalibrated()) ;
        tractionController.resetDefaultAngle();
        telemetry.addLine("IMU calibration finished");
        telemetry.addLine("Ready to start!");
        telemetry.update();
        waitForStart();
        Time.update();
        while(opModeIsActive()) {
            Time.update();
            leftStickX.update(gamepad1.left_stick_x);
            leftStickY.update(gamepad1.left_stick_y);
            rightStickX.update(gamepad1.right_stick_x);

            tractionController.update(leftStickY.get(), leftStickX.get(), rightStickX.get(), telemetry);

            telemetry.update();
        }

    }
}
