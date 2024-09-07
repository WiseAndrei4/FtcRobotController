package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Algo.KeyState;
@Disabled

@TeleOp(name = "IntakeDebugger")
public class IntakeDebugger extends LinearOpMode {

    private static final double NOT_EXTENDED = 0.35; //when the robot picks a pixel
    private static final double EXTENDED = 0.081;

    Servo servo;
    DcMotor motor;
    KeyState up = new KeyState();
    KeyState down = new KeyState();
    KeyState roll = new KeyState();
    KeyState powerUp = new KeyState();
    KeyState powerDown = new KeyState();
    private double power = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class,"servo");
        motor = hardwareMap.get(DcMotor.class, "motor");
        servo.setPosition(NOT_EXTENDED);
        motor.setPower(0.0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()) {
            up.update(gamepad1.dpad_up);
            down.update(gamepad1.dpad_down);
            roll.update(gamepad1.dpad_right);
            powerUp.update(gamepad1.y);
            powerDown.update(gamepad1.a);
            if(up.isKeyJustPressed()) {
                servo.setPosition(NOT_EXTENDED);
            }
            if(down.isKeyJustPressed()) {
                servo.setPosition(EXTENDED);
            }
            if(roll.isKeyJustPressed()) {
                if(motor.getPower()==0.0) {
                    motor.setDirection(DcMotorSimple.Direction.FORWARD);
                    motor.setPower(power);
                }
                else {
                    motor.setPower(0.0);
                }
            }
            if(powerUp.isKeyJustPressed())  power +=0.1;
            if(powerDown.isKeyJustPressed()) power -=0.1;
            telemetry.addData("power ", power);
            telemetry.update();
        }
    }
}
