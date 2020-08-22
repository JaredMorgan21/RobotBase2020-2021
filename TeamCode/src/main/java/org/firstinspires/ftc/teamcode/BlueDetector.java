package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BlueDetector extends OpenCvPipeline {

    public boolean isDetected;
    public colors colors = new colors();

    Mat mask = new Mat();
    Mat imgHSV = new Mat();
    Mat imgBlur = new Mat();
    Mat imgFinal = new Mat();
    Mat hierarchy = new Mat();

    public BlueDetector() {
    }

    @Override
    public Mat processFrame(Mat input) {
        imgFinal.setTo(new Scalar(0,0,0));

        Imgproc.cvtColor(input, imgHSV, Imgproc.COLOR_BGR2HSV);
        Imgproc.GaussianBlur(imgHSV, imgBlur, new Size(1, 1), 20);


        Core.inRange(imgBlur, colors.lower(3), colors.upper(3), mask);

        Core.bitwise_and(input, input, imgFinal, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for(int i = 0; i < contours.size(); i++){

            double area = Imgproc.contourArea(contours.get(i));

            if(area > 100){
                MatOfPoint2f i2f = new MatOfPoint2f(contours.get(i).toArray());
                double peri = Imgproc.arcLength(i2f, true);
                MatOfPoint2f approx = new MatOfPoint2f();
                Imgproc.approxPolyDP(i2f, approx, 0.02 * peri, true);

                Rect rect = Imgproc.boundingRect(approx);

                double w = rect.width;
                double h = rect.height;

                double aspRatio = h/w;
                if(aspRatio > .1 && aspRatio < .3){
                    Imgproc.drawContours(imgFinal, contours, i, new Scalar(255, 0, 0), 3);
                    Imgproc.rectangle(imgFinal, rect, new Scalar(0, 0, 255), 1);
                    isDetected = true;
                }
                else{
                    isDetected = false;
                }
            }
        }

        return imgFinal;
    }
}
