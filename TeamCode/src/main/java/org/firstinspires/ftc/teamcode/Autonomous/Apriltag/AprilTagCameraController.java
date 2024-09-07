package org.firstinspires.ftc.teamcode.Autonomous.Apriltag;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

public class AprilTagCameraController {

    // delay between the time the camera opens and the time we return the camera is ready
    public static final long DELAY = 1000;

    // the camera and the pipeline
    private OpenCvWebcam webcam;
    private AprilTagDetectionPipeline pipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    private double fx = 578.272; // TODO
    private double fy = 578.272; // TODO
    private double cx = 402.145; // TODO
    private double cy = 221.506; // TODO

    // UNITS ARE METERS
    private double tagsize = 0.166; // TODO

    // the time the camera was opened
    private long openTime = -1;

    public AprilTagCameraController(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // create and set the pipeline
        pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        webcam.setPipeline(pipeline);

        // open the camera
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                // open the camera stream
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);

                // set the open time
                synchronized (this) {
                    openTime = System.currentTimeMillis();
                }
            }

            @Override
            public void onError(int errorCode) {
            }

        });
    }

    // get the latest detections
    public List<AprilTagDetection> getLatestDetections() {
        return pipeline.getLatestDetections();
    }

    // returns whether or not the camera is ready to collect data
    public boolean isReady() {
        return openTime != -1 && System.currentTimeMillis() - openTime >= DELAY;
    }

}