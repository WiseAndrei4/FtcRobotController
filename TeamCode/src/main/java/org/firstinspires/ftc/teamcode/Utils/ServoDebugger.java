package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Algo.KeyState;


@Disabled

@TeleOp(name = "ServoDebugger")
public class ServoDebugger extends LinearOpMode {

    Servo servo1;
    Servo servo2;

    private double targetPos1;
    private double targetPos2;
    private double targetPos1o;
    private double targetPos1c;
    private double targetPos2o;
    private double targetPos2c;
    KeyState up1 = new KeyState();
    KeyState down1 = new KeyState();
    KeyState up2 = new KeyState();
    KeyState down2 = new KeyState();
//

    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
       // targetPos1 = servo1.getPosition();
       // targetPos2 = servo2.getPosition();

        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.REVERSE);
        servo2.setPosition(0.13);
        servo1.setPosition(0.13);
        waitForStart();


        //servo1.setPosition(targetPos1o);
      //  servo2.setPosition(targetPos2c);

        while (opModeIsActive()) {
            up1.update(gamepad1.dpad_up);
            down1.update(gamepad1.dpad_down);
            up2.update(gamepad1.y);
            down2.update(gamepad1.a);
            ///DREAPTA  0.61 deschis  ;   0.53 inchis
            ///STANGA    0.02 deschis ;   0.12 inchis
// rotation :  neutral - 0.535   right full - 0.835,  left full -0.25     60degleft - 0.35    60degright - 0.75
// single servo arm rotation :   pickup  - 0.45    release - 0.95
// dual servo arm rotation :

            if (up1.isKeyJustPressed()) {
                targetPos1o +=0.01;
                servo1.setPosition(targetPos1o);
            }
            if (down1.isKeyJustPressed()) {
                targetPos1c -= 0.01;
                servo1.setPosition(targetPos1c);
            }
             if (up2.isKeyJustPressed()) {
                 targetPos2o += 0.01;
                servo2.setPosition(targetPos2o);
            }
            if (down2.isKeyJustPressed()) {
                targetPos2c -= 0.01;
                servo2.setPosition(targetPos2c);
            }
            telemetry.addData("Servo 1 position: ", servo1.getPosition());
            telemetry.addData("Servo 2 position: ", servo2.getPosition());
            telemetry.update();
        }
    }
}

