package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Algo.KeyState;

//0,2 expansion
@Disabled

@TeleOp()
public class LiftDebugger extends LinearOpMode {

    DcMotor lift1;
    DcMotor lift2;

    private double power = 0.0;
    public int position = 0;
    private static KeyState upPower = new KeyState();
    private static KeyState incrementUp = new KeyState();
    private static KeyState incrementDown = new KeyState();
    @Override
    public void runOpMode() throws InterruptedException {
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while(opModeIsActive()){
            upPower.update(gamepad1.dpad_up);
            incrementDown.update(gamepad1.a);
            incrementUp.update(gamepad1.y);
            if(upPower.isKeyJustPressed()) power +=0.1;
            if (incrementDown.isKeyJustPressed() && lift1.getCurrentPosition()>=100){
                position -= 100;
            }
            if (incrementUp.isKeyJustPressed() && lift1.getCurrentPosition() <= 2100){
                position += 100;
            }

            lift1.setTargetPosition(position);
            lift2.setTargetPosition(position);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift1.setPower(power);
            lift2.setPower(power);
            telemetry.addData("lift1 ", lift1.getCurrentPosition());
            telemetry.addData("lift2 ", lift2.getCurrentPosition());
            telemetry.update();
        }

    }

}
