package org.firstinspires.ftc.teamcode.Autonomous.Utils;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ColorDetectionProcessor implements VisionProcessor {

    private Scalar lowerHSV, upperHSV;
    int leftCount,centerCount,rightCount;
    public ColorDetectionProcessor(Scalar lowerHSV, Scalar upperHSV){
        this.lowerHSV=lowerHSV;
        this.upperHSV=upperHSV;
    }
    public static void drawRotatedRect(Mat image, RotatedRect rotatedRect, Scalar color, int thickness) {
        Point[] vertices = new Point[4];
        rotatedRect.points(vertices);
        MatOfPoint points = new MatOfPoint(vertices);
        Imgproc.drawContours(image, (List<MatOfPoint>) points, -1, color, thickness);
    }

    int sampleCount=0;
    int height;
    int width;
    List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
    }

    public enum ZONES {LEFT, CENTER, RIGHT}

    ;
    static ZONES lastZone = ZONES.CENTER;

    @Override
    public Mat processFrame(Mat frame, long time) {
        Mat fail=frame;
        ///HSV = Hue, Saturation, Value
        //Mat output = new Mat();
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
        Core.inRange(frame, lowerHSV, upperHSV, frame);
        leftCount = Core.countNonZero(frame.submat(new Rect(0, height / 5, width / 3, 2 * height / 5)));
        centerCount = Core.countNonZero(frame.submat(new Rect(width / 3, height / 5, width / 3, 2 * height / 5)));
        rightCount = Core.countNonZero(frame.submat(new Rect(2 * width / 3, height / 5, width / 3, 2 * height / 5)));
        int maxi = Math.max(leftCount, Math.max(centerCount, rightCount));
        if (maxi == leftCount) lastZone = ZONES.LEFT;
        else if (maxi == centerCount) lastZone = ZONES.CENTER;
        else lastZone = ZONES.RIGHT;
        if (maxi < 1) {lastZone = null;return fail;}
        Mat kernel1 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2 * 2) + 1, (2 * 2) + 1));
        Imgproc.erode(frame, frame, kernel1);
        Imgproc.dilate(frame, frame, kernel1);
        //Imgproc.erode(frame, frame , kernel);
        //findContours(frame, contours, new Mat(), 0,0,  new Point(3,3));
        //drawContours(frame, contours, 20, new Scalar(100,100,100), 20, 1, new Mat(), 2, new Point(3,3));
        /*List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Mat drawing = Mat.zeros(frame.size(), CvType.CV_8UC3);
        for (int i = 0; i < contours.size(); i++) {
            Scalar color = new Scalar(100, 100, 100);
            Imgproc.drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, new Point());
        }*/
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.GREEN);
        //canvas.drawRect(10, 10, 200, 200, paint);*/
        switch (lastZone) {
            case LEFT: {
                canvas.drawRect(0, 0, width / 3, height, paint);
                break;
            }
            case CENTER: {
                canvas.drawRect(width / 3, 0, width / 3 * 2, height, paint);
                break;
            }
            case RIGHT: {
                canvas.drawRect(width / 3 * 2, 0, width, height, paint);
                break;
            }
            default: {
                canvas.drawRect(0, 0, 100, 100, paint);
                break;
            }
        }
    }

    public ZONES getDetection() {
        return lastZone;
    }

    public int getSampleCount() {
        return sampleCount;
    }

    public void resetSampleCount() {sampleCount=0;}

    public void update(Telemetry telemetry){
        telemetry.addData("Zone 1 ", leftCount);
        telemetry.addData("Zone 2 ", centerCount);
        telemetry.addData("Zone 3 ", rightCount);
    }
}
