package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Algo.KeyState;
@Disabled

@TeleOp(name = "New Motor Debugger")
public class MotorDebuggerNew extends LinearOpMode {
    KeyState incr = new KeyState();
    KeyState decr =  new KeyState();
    @Override
    public void runOpMode(){
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");
        double k = 0.0;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();

        while(opModeIsActive()){

            incr.update(gamepad1.dpad_up);
            decr.update(gamepad1.dpad_down);

            if(incr.isKeyJustPressed() && k<1.0) k+=0.1;
            if(decr.isKeyJustPressed() && k > 0.0) k-=0.1;
            telemetry.addData("Motor power ", k);
            telemetry.update();
            motor.setPower(k);
        }


    }
}
