package org.firstinspires.ftc.teamcode.Autonomous.Utils;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class StackDetectionProcessor extends OpenCvPipeline {

    private static Point lastCenter = new Point(0, 0);

    private final static int SENSITIVITY = 75;
    private final static double STACK_AREA = 20000;
    private final Scalar lowerHSV = new Scalar(0, 255 - SENSITIVITY, 0);
    private final Scalar upperHSV = new Scalar(255, 255, 255);

    @Override
    public void init(Mat firstFrame) {
    }

    public Mat processFrame(Mat frame) {
        try {
            if (frame.empty()) {
                return null;
            }

            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGBA2RGB);
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HLS);
            if (frame.empty()) {
                return null;
            }

            Imgproc.medianBlur(frame, frame, 3);

            Core.inRange(frame, lowerHSV, upperHSV, frame);

            Mat morph = new Mat(3, 3, CvType.CV_32F);
            for (int i = 0; i < 3; i++) {
                morph.put(i, 0, new double[]{1, 1, 1});
            }
            Imgproc.morphologyEx(frame, frame, Imgproc.MORPH_RECT, morph);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_TC89_KCOS);
            if (!contours.isEmpty()) for (MatOfPoint contour : contours) {
                Imgproc.fillPoly(frame, Arrays.asList(contour), new Scalar(255, 255, 255));
                // Imgproc.contourArea(frame);
                // if(Imgproc.contourArea(frame)>500)Imgproc.drawMarker(frame, new Point(0,0),
                // new Scalar(76,76,76));
                /*
                 * double area=Imgproc.contourArea(contour);
                 * if(area>10000){
                 * System.out.println(area);
                 * Rect rect=Imgproc.boundingRect(frame);
                 * Imgproc.rectangle(frame,new Point(rect.x,rect.y),new
                 * Point(rect.x+rect.width,rect.y+rect.height),new Scalar(76,65,150),2);
                 * }
                 */
            }
            else return null;
            contours.clear();
            Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_TC89_KCOS);
            // Imgproc.drawContours(frame, contours, -1, new Scalar(0, 0, 0), 5,
            // Imgproc.LINE_8, hierarchy, 2, new Point());
            if (!contours.isEmpty()) {
                int ind = getBiggestPolygonIndex(contours);
                Rect rect = Imgproc.boundingRect(contours.get(ind));
                // DRAW RECTANGLE:
                Imgproc.rectangle(frame, rect, new Scalar(120, 1, 1));
                lastCenter = new Point(rect.x + (float) rect.width / 2, rect.y + (float) rect.height / 2);
            }
            return null;
        } catch (Exception e) {
            return null;
        }
    }

    private int getBiggestPolygonIndex(final List<MatOfPoint> contours) {
        double maxVal = 0;
        int maxValIdx = 0;
        for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
            double contourArea = Imgproc.contourArea(contours.get(contourIdx));
            if (maxVal < contourArea && contourArea <= STACK_AREA) {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }
        }

        return maxValIdx;
    }

}
