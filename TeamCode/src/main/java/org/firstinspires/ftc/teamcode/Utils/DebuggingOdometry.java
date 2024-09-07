package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled

@TeleOp(name = "Odometry Encoder Debugging")
public class DebuggingOdometry extends LinearOpMode {
    DcMotor motor;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "od");
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("encoder position ", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
