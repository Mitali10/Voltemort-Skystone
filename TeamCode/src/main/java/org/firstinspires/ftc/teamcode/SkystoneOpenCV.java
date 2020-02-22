

package org.firstinspires.ftc.teamcode;

import android.graphics.ImageDecoder;
import android.graphics.ImageFormat;
import android.media.Image;
import android.media.ImageReader;
import android.media.RemoteControlClient;
import android.service.autofill.ImageTransformation;
import android.widget.ImageView;

import com.google.ftcresearch.tfod.util.ImageUtils;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.ImageTarget;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

@TeleOp(name = "Concept: OpenCV Webcam", group = "Skystone")
//@Disabled
public class SkystoneOpenCV extends LinearOpMode {

    OpenCvCamera cam ;

    @Override
    public void runOpMode() {



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);


//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        cam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        cam.openCameraDevice();

        SamplePipeline sp = new SamplePipeline();
        cam.setPipeline(sp);

        telemetry.addData("Going to start streaming", "");
        telemetry.update();


        //cam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        cam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);




        telemetry.addData(">", "Press Play to start opencv");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("location:", sp.location);
                telemetry.update();
                sleep(250);
            }
        }

        cam.stopStreaming();



    }


}
