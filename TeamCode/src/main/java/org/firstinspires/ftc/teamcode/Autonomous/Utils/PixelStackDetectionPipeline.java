package org.firstinspires.ftc.teamcode.Autonomous.Utils;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Controllers.IndividualControllers.EOCVCameraController;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class PixelStackDetectionPipeline implements VisionProcessor {

    public enum Zone {LEFT, CENTER, RIGHT}

    ;

    private final Scalar lowerHSV = new Scalar(0, 0, 75);
    private final Scalar upperHSV = new Scalar(0, 0, 100);

    private Zone lastDetection = Zone.CENTER;
    private int sampleCount = 0;

    private int width, height;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
    }

    public int countIn(Mat frame, int left, int right) {
        int count = 0;

        for (int x = left; x < right; x++)
            for (int y = height / 3; y < height; y++)
                if (frame.get(y, x)[0] >= 0.9)
                    count++;

        return count;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (frame.empty()) {
            return null;
        }

        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
        if (frame.empty()) {
            return null;
        }

        Core.inRange(frame, lowerHSV, upperHSV, frame);
        int leftCount = Core.countNonZero(frame.submat(new Rect(0, height / 3, width / 3, 2 * height / 3)));
        int centerCount = Core.countNonZero(frame.submat(new Rect(width / 3, height / 3, width / 3, 2 * height / 3)));
        int rightCount = Core.countNonZero(frame.submat(new Rect(2 * width / 3, height / 3, width / 3, 2 * height / 3)));

        Mat morph = new Mat(3, 3, CvType.CV_32F);
        for (int i = 0; i < 3; i++) {
            morph.put(i, 0, new double[]{1, 1, 1});
        }
        Imgproc.morphologyEx(frame, frame, Imgproc.MORPH_ERODE, morph);
        //Imgproc.morphologyEx(frame, frame, Imgproc.MORPH_OPEN, morph);

        /*
        int leftCount = countIn(frame, 0, width / 3);
        int centerCount = countIn(frame, width / 3, 2 * width / 3);
        int rightCount = countIn(frame, 2 * width / 3, width);*/

        int maxi = Math.max(centerCount, Math.max(leftCount, rightCount));

        if (maxi < 100)
            return null;

        if (centerCount == maxi) lastDetection = Zone.CENTER;
        else if (leftCount == maxi) lastDetection = Zone.LEFT;
        else if (rightCount == maxi) lastDetection = Zone.RIGHT;
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        /*
        Paint color = new Paint();
        color.setColor(Color.BLUE);

        switch (lastDetection) {
            case LEFT:
                canvas.drawRect(0, 0, onscreenWidth / 3, onscreenHeight, color);
                break;
            case CENTER:
                canvas.drawRect(onscreenWidth / 3, 0, 2 * onscreenWidth / 3, onscreenHeight, color);
                break;
            case RIGHT:
                canvas.drawRect(2 * onscreenWidth / 3, 0, onscreenWidth, onscreenHeight, color);
                break;
        }
        */
    }

    public Zone getDetection() {
        return lastDetection;
    }

    public int getSampleCount() {
        return sampleCount;
    }

    public void resetSampleCount() {
        sampleCount = 0;
    }

    public void calibrateSaturation(EOCVCameraController camera) {
        /// calibrate the saturation with the given hue

    }

}
