package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Algo.KeyState;


@Disabled


@TeleOp(name = "New ServoDebugger")
public class
SingleServoDebugger extends LinearOpMode {

    Servo servo1;
    private double targetPos=0.0;
    KeyState up1 = new KeyState();
    KeyState down1 = new KeyState();
//

    @Override
public void runOpMode() throws InterruptedException {


        //0.23 perpendicular
        //0.5 stack
        //0.58 pickup

        servo1 = hardwareMap.get(Servo.class, "servo");

        servo1.setPosition(0.0);
        waitForStart();

        while (opModeIsActive()) {
            up1.update(gamepad1.dpad_up);
            down1.update(gamepad1.dpad_down);

            if (up1.isKeyJustPressed()) {
                targetPos+=0.01;
            }
            if (down1.isKeyJustPressed()) {
               targetPos-=0.01;
            }
            servo1.setPosition(targetPos);
            telemetry.addData("Servo 1 position: ", servo1.getPosition());
            telemetry.update();
        }
    }
}

// 0.93 not extended servo 4  0.055682 extended
// 0.9  not extended servo 5  0.02 extended