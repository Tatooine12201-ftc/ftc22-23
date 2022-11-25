package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import  org.opencv.core.Mat ;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public abstract class ColorCamera extends OpenCvPipeline {
    Mat mat = new Mat();
    Mat MatPink = new Mat();
    Mat MatGreen = new Mat();
    Mat MatOrange = new Mat();
    Rect Roi = new Rect(new Point(240, 120), new Point(304, 170));
    Mat RoiMat;

    public Mat ProcessFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowerPink = new Scalar(150.0 / 2, 2, 100, 100);
        Scalar upperPink = new Scalar(170.0 / 2, 2, 255, 255);
        Core.inRange(MatPink, lowerPink, upperPink, MatPink);

        Scalar lowerGreen = new Scalar(45.0 / 2, 2, 100, 100);
        Scalar upperGreen = new Scalar(65.0 / 2, 2, 255, 255);
        Core.inRange(MatGreen, lowerGreen, upperGreen, MatGreen);

        Scalar lowerOrange = new Scalar(10.0 / 2, 2, 100, 100);
        Scalar upperOrange = new Scalar(21.0 / 2, 2, 255, 255);
        Core.inRange(MatOrange, lowerOrange, upperOrange, MatOrange);


        RoiMat = mat.submat(Roi);
        MatPink = mat.submat(Roi);
        MatGreen = mat.submat(Roi);
        MatOrange = mat.submat(Roi);
        double PinkValue = Math.round(Core.mean(MatPink).val[2] / 255);
        double GreenValue = Math.round(Core.mean(MatGreen).val[2] / 255);
        double OrangeValue = Math.round(Core.mean(MatOrange).val[2] / 255);
       // RoiMat.release();
        MatPink.release();
        MatGreen.release();
        MatOrange.release();
        mat.release();


        if (GreenValue < PinkValue && OrangeValue < PinkValue) {



        }
        else if (PinkValue < GreenValue && OrangeValue < GreenValue) {


        }
        else {
              

        }

            // if (RoiValue > 150.0 && RoiValue() > 170.0) {
            // "Pink";

        //}
        //else if (RoiValue >45.0&& RoiValue() > 65.0){
        // " Green "
        //}
        //else {
        // Orange
        //}

        return null;

    }
}

   // private double RoiValue() {
      //  return 0;
    //}
//}

