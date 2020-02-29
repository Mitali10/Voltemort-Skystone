package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class SamplePipeline extends OpenCvPipeline {

    Mat mat = new Mat();
    String location = "unknown";
    String alliance = "";
    @Override
    public Mat processFrame(Mat input) {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        /*
         * Draw a simple box around the middle 1/2 of the entire frame
         */

        input.copyTo(mat);

        if (mat.empty()) return input;

        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_RGB2YCrCb);

//        int startRow = 400;
//        int endRow = 480;

//        Mat leftSample = mat.submat(startRow,endRow,80,160);
//        Mat middleSample = mat.submat(startRow,endRow,240,320);
//        Mat rightSample = mat.submat(startRow,endRow,400,480);
//
//        Imgproc.rectangle( mat,new Rect(80,startRow,80,60), new Scalar(255, 0, 0), 4);
//        Imgproc.rectangle( mat,new Rect(240,startRow,80,60), new Scalar(255, 0, 0), 4);
//        Imgproc.rectangle( mat,new Rect(400,startRow,80,60), new Scalar(255, 0, 0), 4);


        int startRow = 135;//200;
        int h = 30;
        int w = 55;

        Mat leftSample, middleSample, rightSample;

        if(alliance == "red") {
            leftSample = mat.submat(startRow,startRow + h,30,30 + w);
            middleSample = mat.submat(startRow,startRow + h,115,115 + w);
            rightSample = mat.submat(startRow,startRow + h,200,200 + w);

            Imgproc.rectangle( mat,new Rect(30, startRow, w, h), new Scalar(255, 0, 0), 4);
            Imgproc.rectangle( mat,new Rect(115, startRow, w, h), new Scalar(255, 0, 0), 4);
            Imgproc.rectangle( mat,new Rect(200, startRow, w, h), new Scalar(255, 0, 0), 4);

        } else {
            leftSample = mat.submat(startRow,startRow + h,60,60 + w);
            middleSample = mat.submat(startRow,startRow + h,145,145 + w);
            rightSample = mat.submat(startRow,startRow + h,230,230 + w);

            Imgproc.rectangle( mat,new Rect(60, startRow, w, h), new Scalar(255, 0, 0), 4);
            Imgproc.rectangle( mat,new Rect(145, startRow, w, h), new Scalar(255, 0, 0), 4);
            Imgproc.rectangle( mat,new Rect(230, startRow, w, h), new Scalar(255, 0, 0), 4);
        }



        double leftVal = Core.sumElems(leftSample).val[2];
        double centreVal = Core.sumElems(middleSample).val[2];
        double rightVal = Core.sumElems(rightSample).val[2];


        if(leftVal > centreVal && leftVal > rightVal)
        {
            location = "left";

        }
        else if(centreVal > rightVal)
        {
            location = "center";
        }
        else
        {
            location = "right";
        }

        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */

        leftSample.release();
        middleSample.release();
        rightSample.release();

        leftSample = null;
        middleSample = null;
        rightSample = null;

        return mat;
    }
}