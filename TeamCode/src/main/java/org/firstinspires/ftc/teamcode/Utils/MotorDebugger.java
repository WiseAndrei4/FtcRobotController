package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled

@TeleOp(name = "MotorDebugger",group = "TeleOp")
public class MotorDebugger extends LinearOpMode {
    private DcMotor motor1,motor2,motor3;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

      //  motor1.setPower(1.0);
        /*motor2.setPower(1.0);
        motor3.setPower(1.0);*/

        while(opModeIsActive()) {
            telemetry.addData("Position 1: ", motor1.getCurrentPosition());
            telemetry.addData("Position 2: ", motor2.getCurrentPosition());
            telemetry.addData("Position 3: ", motor3.getCurrentPosition());
            /*telemetry.addData("Position", motor2.getCurrentPosition());
            telemetry.addData("Position", motor3.getCurrentPosition());*/
            telemetry.update();
        }
    }
}
