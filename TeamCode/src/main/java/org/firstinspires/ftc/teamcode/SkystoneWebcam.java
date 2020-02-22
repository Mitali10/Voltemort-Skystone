

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@TeleOp(name = "Concept: TensorFlow Object Detection Webcam", group = "Skystone")
//@Disabled
public class SkystoneWebcam extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Skystone";
    private static final String LABEL_SECOND_ELEMENT = "Stone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     */
    private static final String VUFORIA_KEY =
            "AUMfooL/////AAABmWMP4b7rnE8PhmtEfcgR/5AhEXZ0rcb0BB83YQKUec7YDlux0IMJBg/2VpYxLTwLx0YkvnpKJQeB0mNJVfiBjLaSkcmeUk/TElajJxeeNOwPFTTod0UigGys8lMtKMn8ICHTB/FjOcfF+pZImjOnZ1J5/U60HP/UGWbUYrFyhuusMVHAavfzP3PmR1xTz+Iodh/LnFNrS73ZXosHwmCZQhWh4NxcxxnBS0CwJ79svlkEkZ7Atq6pv+E7Wzt5xQ8pzrU/wiGenHKxquSIx8MIgapxxULbBhHgriKxSctt5sHL8EI6sn8j0nDCPN4TLo6F5/QatwzKURyNVm21yX8qIr5ILFdO8/raVYUxO596aLZt";


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {

                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            telemetry.addData("Confidence: ",  recognition.getConfidence() );
                            telemetry.addData("Angle to object:", recognition.estimateAngleToObject(AngleUnit.DEGREES));
                        }
                        telemetry.update();
                    }

                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }


    private void initVuforia() {

        try {

            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

            vuforia = ClassFactory.getInstance().createVuforia(parameters);

        } catch(Exception e) {
                telemetry.addData("error in initVuforia", e.toString());
                telemetry.update();
            }
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        tfod.setClippingMargins(55, 100, 50, 120);
    }
}
