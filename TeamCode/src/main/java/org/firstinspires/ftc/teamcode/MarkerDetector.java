package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class MarkerDetector extends OpenCvPipeline{
    public colors colors = new colors();

    Mat imgHSV = new Mat();
    Mat imgBlur = new Mat();

    Mat yellowMask = new Mat();
    Mat pinkMask = new Mat();
    Mat greenMask = new Mat();
    Mat blueMask = new Mat();

    Mat mask = new Mat();
    Mat maskCombo1 = new Mat();
    Mat maskCombo2 = new Mat();
    Mat imgFinal = new Mat();
    Mat imgBlank = new Mat();

    boolean tapped;

    public MarkerDetector() {
    }

    @Override
    public Mat processFrame(Mat input) {
        imgBlank.setTo(new Scalar(0,0,0));

        Imgproc.cvtColor(input, imgHSV, Imgproc.COLOR_BGR2HSV);
        Imgproc.GaussianBlur(imgHSV, imgBlur, new Size(1, 1), 20);

        Core.inRange(imgBlur, colors.lower(0), colors.upper(0), yellowMask);
        Core.inRange(imgBlur, colors.lower(1), colors.upper(1), pinkMask);
        Core.inRange(imgBlur, colors.lower(2), colors.upper(2), greenMask);
        Core.inRange(imgBlur, colors.lower(3), colors.upper(3), blueMask);

        Core.bitwise_or(yellowMask, pinkMask, maskCombo1);
        Core.bitwise_or(greenMask, blueMask, maskCombo2);
        Core.bitwise_or(maskCombo1, maskCombo2, mask);

        if(!tapped){
            imgFinal = imgBlank.clone();
        }
        else{
            imgFinal = input.clone();
        }

        Core.bitwise_and(input, input, imgFinal, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++) {

            double area = Imgproc.contourArea(contours.get(i));

            if (area > 100) {
                MatOfPoint2f i2f = new MatOfPoint2f(contours.get(i).toArray());
                double peri = Imgproc.arcLength(i2f, true);
                MatOfPoint2f approx = new MatOfPoint2f();
                Imgproc.approxPolyDP(i2f, approx, 0.02 * peri, true);

                Rect rect = Imgproc.boundingRect(approx);

                double x = rect.x;
                double y = rect.y;
                double w = rect.width;
                double h = rect.height;

                int middleX = (int) ((int) x + w/2);
                int middleY = (int) ((int) y + h/2);
                Point corner = new Point(x, y);

                double aspRatio = h / w;
                if (aspRatio > .05 && aspRatio < .2) {
                    if(input.get(middleX, middleY) != null){
                        double[] c = imgHSV.get(middleY, middleX);
                        if(c[0] >= 80 && c[0] <= 99){
                            Imgproc.putText(imgFinal, "Yellow", corner, Imgproc.FONT_HERSHEY_COMPLEX, 1.5, new Scalar(255, 255, 0), 3);
                        }
                        else if(c[0] >= 120 && c[0] <= 150){
                            Imgproc.putText(imgFinal, "Pink", corner, Imgproc.FONT_HERSHEY_COMPLEX, 1.5, new Scalar(150, 0, 0), 3);
                        }
                        else if(c[0] >= 45 && c[0] <= 70){
                            Imgproc.putText(imgFinal, "Green", corner, Imgproc.FONT_HERSHEY_COMPLEX, 1.5, new Scalar(0, 255, 0), 3);
                        }
                        else if(c[0] >= 0 && c[0] <= 27){
                            Imgproc.putText(imgFinal, "Blue", corner, Imgproc.FONT_HERSHEY_COMPLEX, 1.5, new Scalar(0, 0, 255), 3);
                        }
                    }
                    Imgproc.drawContours(imgFinal, contours, i, new Scalar(255, 0, 0), 1);
                    Imgproc.rectangle(imgFinal, rect, new Scalar(0, 0, 255), 3);
                }
            }
        }

        return imgFinal;
    }

    @Override
    public void onViewportTapped(){
        tapped = !tapped;
    }
}