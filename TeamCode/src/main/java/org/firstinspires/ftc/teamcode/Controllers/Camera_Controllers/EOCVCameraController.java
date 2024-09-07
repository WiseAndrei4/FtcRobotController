package org.firstinspires.ftc.teamcode.Controllers.Camera_Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.ColorDetectionProcessor;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.StackDetectionProcessor;
import org.firstinspires.ftc.teamcode.Controllers.Controller;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class EOCVCameraController extends Controller {

    public enum Processor { TEAM_PROP, APRIL_TAG, PIXEL_STACK }

    public static double DEBUG_EXPOSURE = 1.0;

    public static double APRIL_TAG_EXPOSURE = 10.0;

    private ColorDetectionProcessor teamPropProcessor;
    private AprilTagProcessor aprilTagProcessor;

    private StackDetectionProcessor pixelStackProcessor;

    private List<VisionProcessor> processors = new ArrayList<>();

    private VisionPortal visionPortal;
    private ExposureControl exposureControl = null;

    public EOCVCameraController(HardwareMap hardwareMap, boolean side) {
        if (side) {
            // BLUE
            //lowerHSV = new Scalar(90, 150, 100);
            //upperHSV = new Scalar(110, 255, 200);
            teamPropProcessor = new ColorDetectionProcessor(new Scalar(90, 100, 20), new Scalar(110, 255, 255));
           // teamPropProcessor = new ColorDetectionProcessor(new Scalar(50, 120, 30), new Scalar(120, 255, 255));
        } else {
            // RED
            //lowerHSV = new Scalar(0, 100, 100);
            //upperHSV = new Scalar(20, 200, 200);
            teamPropProcessor = new ColorDetectionProcessor(new Scalar(0, 70, 15), new Scalar(20, 200, 255));
           // teamPropProcessor = new ColorDetectionProcessor(new Scalar(0, 60, 0), new Scalar(15, 190, 255));
        }

        pixelStackProcessor=new StackDetectionProcessor();

        processors.add(teamPropProcessor);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        builder.setCamera(webcamName);

        for(VisionProcessor processor : processors) {
            builder.addProcessor(processor);
        }

        visionPortal = builder.build();
        visionPortal.resumeStreaming();
        disableAll();
    }

    public void Close() {
        visionPortal.close();
    }

    public void PauseStream() {
        visionPortal.stopStreaming();
    }

    public void ResumeStream() {
        visionPortal.resumeStreaming();
    }

    public ColorDetectionProcessor getTeamPropProcessor() {
        return teamPropProcessor;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }

    public void disableAll() {
        try {
            for (VisionProcessor processor : processors) {
                visionPortal.setProcessorEnabled(processor, false);
            }
        } catch(Exception e) {

        }
    }

    public void checkControls() {
        try {
            if (exposureControl == null) {
                exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            }
        } catch(Exception e) {

        }
    }

    public void setProcessor(Processor processor, Telemetry telemetry) {
        disableAll();
        checkControls();

        try {
            switch (processor) {
                case TEAM_PROP: {
                    telemetry.addLine("YOOOOOOOO!");
                    exposureControl.setMode(ExposureControl.Mode.AperturePriority);
                    visionPortal.setProcessorEnabled(teamPropProcessor, true);
                    break;
                }
                /*case APRIL_TAG: {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    exposureControl.setExposure((long) APRIL_TAG_EXPOSURE, TimeUnit.MILLISECONDS);
                    visionPortal.setProcessorEnabled(aprilTagProcessor, true);
                    break;
                }
                case PIXEL_STACK:{

                    break;
                }*/
            }
        } catch(Exception e) {

        }
    }

    public void debug(Telemetry telemetry) {
        checkControls();

        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure((long) DEBUG_EXPOSURE, TimeUnit.MILLISECONDS);

        telemetry.addData("Exposure", exposureControl.getExposure(TimeUnit.MILLISECONDS));
    }
}
