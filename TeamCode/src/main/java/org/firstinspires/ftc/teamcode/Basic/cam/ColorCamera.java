package org.firstinspires.ftc.teamcode.Basic.cam;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorCamera extends OpenCvPipeline {
    Mat mat = new Mat();
    Mat pinkMat = new Mat();
    Mat greenMat = new Mat();
    Mat orangeMat = new Mat();
    // 320, 240
    //Rect leftROI = new Rect(new Point(0, 0), new Point(320 / 3.0, 240));
    Rect midROI = new Rect(new Point(320 / 3.0, 240 /3.0), new Point(2 * 320 / 4.0, 2 * 240 /4.0));
    //Rect rightROI = new Rect(new Point(2 * 320 / 3.0, 0), new Point(320, 240));

    private zone result = null;
    private Telemetry telemetry;

    public ColorCamera(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowerPink = new Scalar(100 / 2.0, 100, 100);
        Scalar upperPink = new Scalar(140 / 2.0, 255, 255);
        Core.inRange(mat, lowerPink, upperPink, pinkMat);


        Scalar lowerGreen = new Scalar(40 / 2.0, 100, 100);
        Scalar upperGreen = new Scalar(80 / 2.0, 255, 255);
        Core.inRange(mat, lowerGreen, upperGreen, greenMat);


        Scalar lowerOrange = new Scalar(10 / 2.0, 100, 100);
        Scalar upperOrange = new Scalar(30 / 2.0, 255, 255);
        Core.inRange(mat, lowerOrange, upperOrange, orangeMat);

        greenMat = greenMat.submat(midROI);
        orangeMat = orangeMat.submat(midROI);
        pinkMat = pinkMat.submat(midROI);



        double pinkValue = Core.sumElems(pinkMat).val[0];
        double greenValue = Core.sumElems(greenMat).val[0];
        double orangeValue = Core.sumElems(orangeMat).val[0];
        telemetry.addData("pink", pinkValue);
        telemetry.addData("green", greenValue);
        telemetry.addData("orange", orangeValue);

        double maxValue = Math.max(pinkValue, Math.max(greenValue, orangeValue));
        Scalar pinkColor = new Scalar(130 /2.0, 255, 250);
        Scalar greenColor = new Scalar(60 /2.0, 255, 250);
        Scalar orangeColor = new Scalar(20 /2.0, 255, 250);
        if (maxValue == pinkValue) {
            result = zone.A;
            Imgproc.rectangle(input, midROI, pinkColor);
        } else if (maxValue == greenValue) {
            result = zone.C;
            Imgproc.rectangle(input, midROI, greenColor);
        } else {
            result = zone.B;
            Imgproc.rectangle(input, midROI, orangeColor);
        }
                              telemetry.addData("zone", result.toString().toLowerCase());
        telemetry.update();



        return input;
    }

    public zone getResult() {
        // CountDownLatch
        while (result == null);
        return result;
    }
}