package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

@Autonomous(name = "RoverTest:RoverAutonomousBase", group = "RoverTest")
@Disabled
public class RoverAutonomousBase extends LinearOpMode {

    enum Task {LAND, BACK_UP, SCAN, RUN_TO_DEPOT, RUN_TO_CRATER, PUSH_MINERAL, CUBE_LEFT, CUBE_CENTER, CUBE_RIGHT, NO_CUBE, DROP_MARKER, HALT, RESET, RUN_RIGHT, SCAN_LEFT, RUN_BACK, MOVE, RUN_STRAIGHT_2};

    Task nextTask;

    DcMotor wheelFrontRight, wheelFrontLeft, wheelRearRight, wheelRearLeft, linearMotor, linearMotor2, armMotor, linearSlideMotor;
    DigitalChannel limitSwitch1;
    Servo markerServo, markerServoSide;

    final double RUN_STRAIGHT_SPEED = 0.6;
    final double TURNING_SPEED = 0.35;
    final double STRAFE_SPEED = 0.1;
    final double BACKUP_SPEED = 0.6;


    final int ENCODER_CPR = 1680;  //1120 for Neverest 40s
    final double GEAR_RATIO = 1;   //changed 1/29
    final int WHEEL_DIA = 6;  //in inches

    BNO055IMU imu;
    Orientation angles;

    static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    static final String VUFORIA_KEY = "AQ4Va/L/////AAAAmY8z7ZmR3kS9h4kjXYc/4zNVg3Zlto5QVTHJAKj9x7uf4yOcJ4OhNe4DfKQwrK20PsJyBMH8Di37xUsOzfuXh21jTiJxjzwpkgLO2EXMsirLNGqSsIGGLiIICDPmakq8qNcprbgCA1/YmbbtMGGlqeYtvR87tEs5z1gCtK0ljbMlqskDkqqIGi7JGbXa/RHDxxzaYoqOyTrV37vd45KkHs5KFVSX0zyH38pzmV3bjW81v6Za9A7Hndv8LqvE+qmp7IkogttT8+vefVTDSMLhjv5d5I9YyR/s860rByRv7IQqFsMSxlbkdFe3uIn4dI/FkVPpoZLAo/MoaaAJIvdVJ4Y0Vz8DgyGSAFuBJF2GMNY+";
    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;

    double distanceTraveled = 0;
    ElapsedTime runtime = new ElapsedTime();

//    RoverAutonomousBase(){
//        setupAllHardware();
//    }

    @Override
    public void runOpMode() {

        // Implement this is sub-classes .

    }

    public void setupAllHardware() {

        //wheels

        wheelFrontRight = hardwareMap.get(DcMotor.class, "wheelFrontRight");
        wheelFrontLeft = hardwareMap.get(DcMotor.class, "wheelFrontLeft");
        wheelRearRight = hardwareMap.get(DcMotor.class, "wheelRearRight");
        wheelRearLeft = hardwareMap.get(DcMotor.class, "wheelRearLeft");

        wheelFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRearLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheelFrontRight.setDirection(DcMotor.Direction.REVERSE);
        wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRearRight.setDirection(DcMotor.Direction.REVERSE);
        wheelRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wheelRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //motors

        linearMotor = hardwareMap.dcMotor.get("linearMotor");
        linearMotor.setDirection(DcMotor.Direction.FORWARD);
        linearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearMotor2 = hardwareMap.dcMotor.get("linearMotor2");
        linearMotor2.setDirection(DcMotor.Direction.FORWARD);
        linearMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearSlideMotor = hardwareMap.dcMotor.get("linearSlideMotor");
        linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor = hardwareMap.dcMotor.get("armMotor");
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //servos

        markerServo = hardwareMap.get(Servo.class, "markerServo");
        markerServoSide = hardwareMap.get(Servo.class, "markerServoSide");
        markerServo.setPosition(0.4);
        markerServoSide.setPosition(0.0);

        //sensors

        limitSwitch1 = hardwareMap.get(DigitalChannel.class, "mag1");   //top
        limitSwitch1.setMode(DigitalChannel.Mode.INPUT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.useExternalCrystal = true;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

    }

    public void encoderDrive(double speed, double distance, double timeoutS) {
        int newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newRearLeftTarget = wheelRearLeft.getCurrentPosition() + getEncoderPulse(distance);
            newRearRightTarget = wheelRearRight.getCurrentPosition() + getEncoderPulse(distance);
            newFrontLeftTarget = wheelFrontLeft.getCurrentPosition() + getEncoderPulse(distance);
            newFrontRightTarget = wheelFrontRight.getCurrentPosition() + getEncoderPulse(distance);

            wheelRearLeft.setTargetPosition(newRearLeftTarget);
            wheelRearRight.setTargetPosition(newRearRightTarget);
            wheelFrontLeft.setTargetPosition(newFrontLeftTarget);
            wheelFrontRight.setTargetPosition(newFrontRightTarget);


            // Turn On RUN_TO_POSITION
            wheelRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            wheelRearLeft.setPower(abs(speed));
            wheelRearRight.setPower(abs(speed));
            wheelFrontLeft.setPower(abs(speed));
            wheelFrontRight.setPower(abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (wheelRearLeft.isBusy() && wheelRearRight.isBusy() && wheelFrontLeft.isBusy() && wheelFrontRight.isBusy())) {
                /*
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        wheelLeft.getCurrentPosition(),
                        wheelRight.getCurrentPosition());
                telemetry.update();
				*/

            }

            // Stop all motion;
            wheelRearLeft.setPower(0);
            wheelRearRight.setPower(0);
            wheelFrontLeft.setPower(0);
            wheelFrontRight.setPower(0);


            // Turn off RUN_TO_POSITION
            wheelRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move
        }
    }

    public boolean encoderForwardSearching(double speed, double distance, double timeoutS, double confidence) {
        int newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget;
        boolean success = false;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newRearLeftTarget = wheelRearLeft.getCurrentPosition() + getEncoderPulse(distance);
            newRearRightTarget = wheelRearRight.getCurrentPosition() + getEncoderPulse(distance);
            newFrontLeftTarget = wheelFrontLeft.getCurrentPosition() + getEncoderPulse(distance);
            newFrontRightTarget = wheelFrontRight.getCurrentPosition() + getEncoderPulse(distance);

            wheelRearLeft.setTargetPosition(newRearLeftTarget);
            wheelRearRight.setTargetPosition(newRearRightTarget);
            wheelFrontLeft.setTargetPosition(newFrontLeftTarget);
            wheelFrontRight.setTargetPosition(newFrontRightTarget);


            // Turn On RUN_TO_POSITION
            wheelRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            wheelRearLeft.setPower(abs(speed));
            wheelRearRight.setPower(abs(speed));
            wheelFrontLeft.setPower(abs(speed));
            wheelFrontRight.setPower(abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (wheelRearLeft.isBusy() && wheelRearRight.isBusy() && wheelFrontLeft.isBusy() && wheelFrontRight.isBusy())) {


                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions(); //getRecognitions();


                if (updatedRecognitions != null && updatedRecognitions.size() == 1 && updatedRecognitions.get(0).getLabel().equalsIgnoreCase(LABEL_GOLD_MINERAL) && updatedRecognitions.get(0).getConfidence() > confidence) {       //0.62

                    distanceTraveled = getStrafedInchesFromEncoder(wheelFrontLeft.getCurrentPosition());
                    telemetry.addData("Object Found ", updatedRecognitions.get(0).getLabel() + " " +  updatedRecognitions.get(0).getConfidence());
                    telemetry.update();
                    success = true;
                    break;

                }


                /*
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        wheelLeft.getCurrentPosition(),
                        wheelRight.getCurrentPosition());
                telemetry.update();
				*/

            }




            tfod.deactivate();
            tfod.shutdown();

            // Stop all motion;
            wheelRearLeft.setPower(0);
            wheelRearRight.setPower(0);
            wheelFrontLeft.setPower(0);
            wheelFrontRight.setPower(0);


            // Turn off RUN_TO_POSITION
            wheelRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move
        }
        return success;
    }


    public boolean encoderStrafeLeftSearching(double speed, double distance, double timeoutS, double confidence) {
        int newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget;
        boolean success = false;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            double strafeDistance = abs(distance) * sqrt(2);
            // Determine new target position, and pass to motor controller
            newRearLeftTarget = wheelRearLeft.getCurrentPosition() + getEncoderPulse(strafeDistance);
            newRearRightTarget = wheelRearRight.getCurrentPosition() - getEncoderPulse(strafeDistance);
            newFrontLeftTarget = wheelFrontLeft.getCurrentPosition() - getEncoderPulse(strafeDistance);
            newFrontRightTarget = wheelFrontRight.getCurrentPosition() + getEncoderPulse(strafeDistance);

            wheelRearLeft.setTargetPosition(newRearLeftTarget);
            wheelRearRight.setTargetPosition(newRearRightTarget);
            wheelFrontLeft.setTargetPosition(newFrontLeftTarget);
            wheelFrontRight.setTargetPosition(newFrontRightTarget);


            // Turn On RUN_TO_POSITION
            wheelRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            wheelRearLeft.setPower(abs(speed));
            wheelRearRight.setPower(abs(speed));
            wheelFrontLeft.setPower(abs(speed));
            wheelFrontRight.setPower(abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (wheelRearLeft.isBusy() && wheelRearRight.isBusy() && wheelFrontLeft.isBusy() && wheelFrontRight.isBusy())) {

                List<Recognition> updatedRecognitions = tfod.getRecognitions();//getUpdatedRecognitions();

                if (updatedRecognitions != null && updatedRecognitions.size() == 1 && updatedRecognitions.get(0).getLabel().equalsIgnoreCase(LABEL_GOLD_MINERAL) && updatedRecognitions.get(0).getConfidence() > confidence) {//0.62

                    distanceTraveled = getStrafedInchesFromEncoder(wheelFrontLeft.getCurrentPosition());
                    telemetry.addData("Yohoo! Gold Found : ", distanceTraveled);
                    telemetry.update();
                    success = true;
                    break;

                }


                /*
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        wheelLeft.getCurrentPosition(),
                        wheelRight.getCurrentPosition());
                telemetry.update();
				*/
            }

            tfod.deactivate();
            tfod.shutdown();
            // Stop all motion;
            wheelRearLeft.setPower(0);
            wheelRearRight.setPower(0);
            wheelFrontLeft.setPower(0);
            wheelFrontRight.setPower(0);

            // Turn off RUN_TO_POSITION
            wheelRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move

        }
        return success;
    }

    public void encoderStrafeLeft(double speed, double distance, double timeoutS) {
        int newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            double strafeDistance = abs(distance) * sqrt(2);
            // Determine new target position, and pass to motor controller
            newRearLeftTarget = wheelRearLeft.getCurrentPosition() + getEncoderPulse(strafeDistance);
            newRearRightTarget = wheelRearRight.getCurrentPosition() - getEncoderPulse(strafeDistance);
            newFrontLeftTarget = wheelFrontLeft.getCurrentPosition() - getEncoderPulse(strafeDistance);
            newFrontRightTarget = wheelFrontRight.getCurrentPosition() + getEncoderPulse(strafeDistance);

            wheelRearLeft.setTargetPosition(newRearLeftTarget);
            wheelRearRight.setTargetPosition(newRearRightTarget);
            wheelFrontLeft.setTargetPosition(newFrontLeftTarget);
            wheelFrontRight.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            wheelRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            wheelRearLeft.setPower(abs(speed));
            wheelRearRight.setPower(abs(speed));
            wheelFrontLeft.setPower(abs(speed));
            wheelFrontRight.setPower(abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (wheelRearLeft.isBusy() && wheelRearRight.isBusy() && wheelFrontLeft.isBusy() && wheelFrontRight.isBusy())) {


                /*
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        wheelLeft.getCurrentPosition(),
                        wheelRight.getCurrentPosition());
                telemetry.update();
				*/
            }

            // Stop all motion;
            wheelRearLeft.setPower(0);
            wheelRearRight.setPower(0);
            wheelFrontLeft.setPower(0);
            wheelFrontRight.setPower(0);


            // Turn off RUN_TO_POSITION
            wheelRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move
        }
    }


    public void encoderStrafeLeft(double speed, double timeoutS) {
        int newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            runtime.reset();
            wheelRearLeft.setPower(abs(speed));
            wheelRearRight.setPower(-(abs(speed)));
            wheelFrontLeft.setPower(-(abs(speed)));
            wheelFrontRight.setPower(abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)) {
                //Doing nothing !!
            }

            // Stop all motion;
            wheelRearLeft.setPower(0);
            wheelRearRight.setPower(0);
            wheelFrontLeft.setPower(0);
            wheelFrontRight.setPower(0);


            sleep(250);   // optional pause after each move
        }
    }


    public void encoderStrafeRight(double speed, double distance, double timeoutS) {
        int newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            double strafeDistance = abs(distance) * sqrt(2);
            // Determine new target position, and pass to motor controller
            newRearLeftTarget = wheelRearLeft.getCurrentPosition() - getEncoderPulse(strafeDistance);
            newRearRightTarget = wheelRearRight.getCurrentPosition() + getEncoderPulse(strafeDistance);
            newFrontLeftTarget = wheelFrontLeft.getCurrentPosition() + getEncoderPulse(strafeDistance);
            newFrontRightTarget = wheelFrontRight.getCurrentPosition() - getEncoderPulse(strafeDistance);

            wheelRearLeft.setTargetPosition(newRearLeftTarget);
            wheelRearRight.setTargetPosition(newRearRightTarget);
            wheelFrontLeft.setTargetPosition(newFrontLeftTarget);
            wheelFrontRight.setTargetPosition(newFrontRightTarget);


            // Turn On RUN_TO_POSITION
            wheelRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            wheelRearLeft.setPower(abs(speed));
            wheelRearRight.setPower(abs(speed));
            wheelFrontLeft.setPower(abs(speed));
            wheelFrontRight.setPower(abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (wheelRearLeft.isBusy() && wheelRearRight.isBusy() && wheelFrontLeft.isBusy() && wheelFrontRight.isBusy())) {
                /*
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        wheelLeft.getCurrentPosition(),
                        wheelRight.getCurrentPosition());
                telemetry.update();
				*/
            }

            // Stop all motion;
            wheelRearLeft.setPower(0);
            wheelRearRight.setPower(0);
            wheelFrontLeft.setPower(0);
            wheelFrontRight.setPower(0);


            // Turn off RUN_TO_POSITION
            wheelRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move
        }
    }

    public void encoderStrafeRight(double speed, double timeoutS) {
        int newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            runtime.reset();
            wheelRearLeft.setPower(-abs(speed));
            wheelRearRight.setPower(abs(speed));
            wheelFrontLeft.setPower(abs(speed));
            wheelFrontRight.setPower(-abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)) {
                //Doing nothing !!
            }

            // Stop all motion;
            wheelRearLeft.setPower(0);
            wheelRearRight.setPower(0);
            wheelFrontLeft.setPower(0);
            wheelFrontRight.setPower(0);


            sleep(250);   // optional pause after each move
        }
    }

    public void turnRight(int angle) {
        while ((1 >= getHeading()) || getHeading() >= angle) {      //angle is 270 for 90 deg turn (315 for 45 deg)

            wheelFrontLeft.setPower(TURNING_SPEED);
            wheelRearLeft.setPower(TURNING_SPEED);
            wheelFrontRight.setPower(-TURNING_SPEED);
            wheelRearRight.setPower(-TURNING_SPEED);

        }

        wheelFrontLeft.setPower(0);
        wheelRearLeft.setPower(0);
        wheelFrontRight.setPower(0);
        wheelRearRight.setPower(0);

        //requestOpModeStop();
        sleep(250);   // optional pause after each move

    }

    public void turnLeft(int angle) {
        while (getHeading() > 345 || getHeading() <= angle) {         //angle is 90 for 90 deg turns (45 for 45 deg)

            wheelFrontLeft.setPower(-TURNING_SPEED);
            wheelRearLeft.setPower(-TURNING_SPEED);
            wheelFrontRight.setPower(TURNING_SPEED);
            wheelRearRight.setPower(TURNING_SPEED);

            telemetry.addData("heading:", getHeading());
            telemetry.update();

        }

        wheelFrontLeft.setPower(0);
        wheelRearLeft.setPower(0);
        wheelFrontRight.setPower(0);
        wheelRearRight.setPower(0);

        //requestOpModeStop();
        sleep(250);   // optional pause after each move

    }

    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle + 360) % 360;
    }

    public int getEncoderPulse(double dist) {

        return (int) ((dist / (Math.PI * WHEEL_DIA)) * ENCODER_CPR * GEAR_RATIO);
    }

    public double getStrafedInchesFromEncoder(int i) {
        return (i * Math.PI * WHEEL_DIA) / (ENCODER_CPR * GEAR_RATIO * Math.sqrt(2));
    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
      //  tfodParameters.useObjectTracker = false;
        // tfodParameters.minimumConfidence = 0.75;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL);
    }


    public void climbDown() {

        if (opModeIsActive()) {

            while (limitSwitch1.getState()) {
                linearMotor.setPower(-0.8);
                linearMotor2.setPower(-0.8);
            }
            linearMotor.setPower(0);
            linearMotor2.setPower(0);
        }
    }



}

