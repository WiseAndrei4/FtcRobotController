package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Disabled
@TeleOp(name="Sensor Debugger")
public class SensorDebugger extends LinearOpMode {

    @Override
    public void runOpMode() {

        AnalogInput chanalog0 = hardwareMap.get(AnalogInput.class, "CHanalog0");
        AnalogInput chanalog1 = hardwareMap.get(AnalogInput.class, "CHanalog1");
        AnalogInput chanalog2 = hardwareMap.get(AnalogInput.class, "CHanalog2");
        AnalogInput chanalog3 = hardwareMap.get(AnalogInput.class, "CHanalog3");

        AnalogInput ehanalog0 = hardwareMap.get(AnalogInput.class, "EHanalog0");
        AnalogInput ehanalog1 = hardwareMap.get(AnalogInput.class, "EHanalog1");
        AnalogInput ehanalog2 = hardwareMap.get(AnalogInput.class, "EHanalog2");
        AnalogInput ehanalog3 = hardwareMap.get(AnalogInput.class, "EHanalog3");

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("CHanalog0", chanalog0.getVoltage());
            telemetry.addData("CHanalog1", chanalog1.getVoltage());
            telemetry.addData("CHanalog2", chanalog2.getVoltage());
            telemetry.addData("CHanalog3", chanalog3.getVoltage());

            telemetry.addData("EHanalog0", ehanalog0.getVoltage());
            telemetry.addData("EHanalog1", ehanalog1.getVoltage());
            telemetry.addData("EHanalog2", ehanalog2.getVoltage());
            telemetry.addData("EHanalog3", ehanalog3.getVoltage());
            telemetry.update();
        }

    }

}
