package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

@Autonomous(name = "SkystoneAutonomousBaseOld", group = "Skystone")
@Disabled
public class SkystoneAutonomousBaseOld extends LinearOpMode {

    enum Task {HALT, MOVE, APPROACH_STONES, SEARCH, LIFT_SKYSTONE, RETURN_TO_STONES, DROP_SKYSTONE2, LIFT_SKYSTONE2, DRIVE_TO_FOUNDATION, DROP_SKYSTONE, MOVE_FOUNDATION, PARK}

    Task nextTask;

    DcMotor wheelFrontRight, wheelFrontLeft, wheelRearRight, wheelRearLeft, leftSlide, rightSlide, topActuator;
    Servo gripperServo, turnServo, foundationServo1, foundationServo2;
    ColorSensor colorSensor;
    private DistanceSensor rangeSensor;

    static final double GRIPPER_CLOSE_POS = 0.65;
    static final double GRIPPER_OPEN_POS = 0.0;
    static final double FOUNDATION_UP = 0.0;
    static final double FOUNDATION_DOWN = 1.0;

    BNO055IMU imu;
    Orientation angles;
    final double TURNING_SPEED = 0.1;
    final double BACKUP_SPEED = 0.6;

    final double ENCODER_CPR = 383.6; //for goBuilda 435 rmp 753.2; // 188.3; // for goBUILDA 223rpm // last year - 1680;  //1120 for Neverest 40s
    final double GEAR_RATIO = 2;   //changed 1/29
    final double WHEEL_DIA = 3.93;  //in inches
    double currentAngle = 0;

    final double ENCODER_CPR_LINEAR_ACTUATOR = 145.6;
    final double ROTATION_PER_IN_LINEAR_ACTUATOR = 3.175;


    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_STONE = "Stone";
    private static final String LABEL_SKYSTONE = "Skystone";
    private static final String VUFORIA_KEY =
            "AUMfooL/////AAABmWMP4b7rnE8PhmtEfcgR/5AhEXZ0rcb0BB83YQKUec7YDlux0IMJBg/2VpYxLTwLx0YkvnpKJQeB0mNJVfiBjLaSkcmeUk/TElajJxeeNOwPFTTod0UigGys8lMtKMn8ICHTB/FjOcfF+pZImjOnZ1J5/U60HP/UGWbUYrFyhuusMVHAavfzP3PmR1xTz+Iodh/LnFNrS73ZXosHwmCZQhWh4NxcxxnBS0CwJ79svlkEkZ7Atq6pv+E7Wzt5xQ8pzrU/wiGenHKxquSIx8MIgapxxULbBhHgriKxSctt5sHL8EI6sn8j0nDCPN4TLo6F5/QatwzKURyNVm21yX8qIr5ILFdO8/raVYUxO596aLZt";
    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;

    double distanceTraveled = 0;
    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        // Implement this is sub-classes.

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


        leftSlide = hardwareMap.dcMotor.get("leftSlide");
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightSlide = hardwareMap.dcMotor.get("rightSlide");
        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topActuator = hardwareMap.dcMotor.get("topActuator");
        topActuator.setDirection(DcMotor.Direction.FORWARD);
        topActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
        gripperServo.setPosition(GRIPPER_CLOSE_POS);    // 1 is open, 0.25 is gripped

        turnServo = hardwareMap.get(Servo.class, "turnServo");
        turnServo.setPosition(0.5);

        foundationServo1 = hardwareMap.get(Servo.class, "foundationServo1");
        foundationServo1.setPosition(0.0); // up pos

        foundationServo2 = hardwareMap.get(Servo.class, "foundationServo2");
        foundationServo2.setPosition(1.0);  //up pos

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.useExternalCrystal = true;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(false);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calibration status:", imu.getCalibrationStatus().toString());
        telemetry.update();


        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

    }

    public void encoderDriveForwardUntilDistance(double speed, double distanceTo, double timeoutS) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            runtime.reset();
            wheelRearLeft.setPower(abs(speed));
            wheelRearRight.setPower(abs(speed));
            wheelFrontLeft.setPower(abs(speed));
            wheelFrontRight.setPower(abs(speed));

            while (opModeIsActive() && (rangeSensor.getDistance(DistanceUnit.INCH) > distanceTo) && (runtime.seconds() < timeoutS)) {
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

    public void encoderDriveActuator(double speed, double distance, double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = topActuator.getCurrentPosition() - getEncoderPulseActuator(distance);

            topActuator.setTargetPosition(newTarget);

            topActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            topActuator.setPower(abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (topActuator.isBusy())) {
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
            topActuator.setPower(0);

            // Turn off RUN_TO_POSITION
            topActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public boolean encoderStrafeLeftColor(double speed, double distance, double timeoutS, double colorThresh) {
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

                telemetry.addData("blue ", colorSensor.blue() );
                telemetry.update();

                if (colorSensor.blue() > colorThresh) {
                    telemetry.addData("Passed blue ", "" );
                    telemetry.update();
                    success = true;
                    break;

                }

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
        return success;
    }


    public boolean encoderStrafeLeftSearching(double speed, double distance, double timeoutS, double confidence) {
        int newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget;
        boolean success = false;
        double start = wheelFrontLeft.getCurrentPosition();

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

                if (updatedRecognitions != null && updatedRecognitions.size() == 1 && updatedRecognitions.get(0).getLabel().equalsIgnoreCase(LABEL_SKYSTONE) && updatedRecognitions.get(0).getConfidence() > confidence) {

                    distanceTraveled = getStrafedInchesFromEncoder((wheelFrontLeft.getCurrentPosition() - start));
                    telemetry.addData("Yohoo! Skystone Found : ", updatedRecognitions.get(0).getConfidence());
                    telemetry.update();
                    success = true;
                    break;

                }

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
    public boolean encoderStrafeRightColor(double speed, double distance, double timeoutS, double colorThresh) {
        int newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget;
        boolean success = false;

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

                telemetry.addData("red ", colorSensor.red() );
                telemetry.update();

                if (colorSensor.blue() > colorThresh) {
                    telemetry.addData("Passed red ", "" );
                    telemetry.update();
                    success = true;
                    break;

                }

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

        return success;
    }

    public boolean encoderStrafeRightSearching(double speed, double distance, double timeoutS, double confidence) {
        int newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget;
        boolean success = false;
        double start = wheelFrontLeft.getCurrentPosition();

        boolean res = checkRecognitions(confidence);
        if(res) {
            distanceTraveled = getStrafedInchesFromEncoder((wheelFrontLeft.getCurrentPosition() - start));
            telemetry.update();
            tfod.deactivate();
            tfod.shutdown();
            return true;
        }

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

                List<Recognition> updatedRecognitions = tfod.getRecognitions();//getUpdatedRecognitions();

                if (updatedRecognitions != null && updatedRecognitions.size() == 1 && updatedRecognitions.get(0).getLabel().equalsIgnoreCase(LABEL_SKYSTONE) && updatedRecognitions.get(0).getConfidence() > confidence) {//0.62

                    distanceTraveled = getStrafedInchesFromEncoder((wheelFrontLeft.getCurrentPosition() - start));
                    telemetry.addData("Yohoo! Skystone Found : ", updatedRecognitions.get(0).getConfidence());
                    telemetry.update();
                    success = true;
                    break;

                }

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

    public boolean checkRecognitions(double confidence) {
        sleep(400);


        List<Recognition> updatedRecognitions = tfod.getRecognitions();//getUpdatedRecognitions();

        if (updatedRecognitions != null) {
            for (Recognition recognition : updatedRecognitions) {
                telemetry.addData("Object found: " + recognition.getLabel(), recognition.getConfidence());
                telemetry.update();
                if (recognition.getLabel().equalsIgnoreCase(LABEL_SKYSTONE) && recognition.getConfidence() > confidence) {
                    telemetry.addData("Yohoo! Skystone Found : ", recognition.getConfidence());
                    telemetry.update();
                    return true;
                }

            }
        }


        return false;
    }


    public void encoderStrafeLeft(double speed, double distance, double timeoutS) {
        int newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // double strafeDistance = abs(distance) * sqrt(2);
            double strafeDistance = abs(distance);

            // Determine new target position, and pass to motor controller
//            newRearLeftTarget = wheelRearLeft.getCurrentPosition() + getEncoderPulse(strafeDistance);
//            newRearRightTarget = wheelRearRight.getCurrentPosition() - getEncoderPulse(strafeDistance);
//            newFrontLeftTarget = wheelFrontLeft.getCurrentPosition() - getEncoderPulse(strafeDistance);
//            newFrontRightTarget = wheelFrontRight.getCurrentPosition() + getEncoderPulse(strafeDistance);

            newRearLeftTarget = wheelRearLeft.getCurrentPosition() + getEncoderPulseStrafe(strafeDistance);
            newRearRightTarget = wheelRearRight.getCurrentPosition() - getEncoderPulseStrafe(strafeDistance);
            newFrontLeftTarget = wheelFrontLeft.getCurrentPosition() - getEncoderPulseStrafe(strafeDistance);
            newFrontRightTarget = wheelFrontRight.getCurrentPosition() + getEncoderPulseStrafe(strafeDistance);

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

            //double strafeDistance = abs(distance) * sqrt(2);
            double strafeDistance = abs(distance);
            // Determine new target position, and pass to motor controller
//            newRearLeftTarget = wheelRearLeft.getCurrentPosition() - getEncoderPulse(strafeDistance);
//            newRearRightTarget = wheelRearRight.getCurrentPosition() + getEncoderPulse(strafeDistance);
//            newFrontLeftTarget = wheelFrontLeft.getCurrentPosition() + getEncoderPulse(strafeDistance);
//            newFrontRightTarget = wheelFrontRight.getCurrentPosition() - getEncoderPulse(strafeDistance);

            newRearLeftTarget = wheelRearLeft.getCurrentPosition() - getEncoderPulseStrafe(strafeDistance);
            newRearRightTarget = wheelRearRight.getCurrentPosition() + getEncoderPulseStrafe(strafeDistance);
            newFrontLeftTarget = wheelFrontLeft.getCurrentPosition() + getEncoderPulseStrafe(strafeDistance);
            newFrontRightTarget = wheelFrontRight.getCurrentPosition() - getEncoderPulseStrafe(strafeDistance);

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

        double heading = getHeading();
        double targetAngle = currentAngle - angle;

        telemetry.addData("turn Right heading beg:", heading);
        telemetry.update();

        if(targetAngle < 0) {
            targetAngle += 360;
            while(getHeading() < 355) {
                telemetry.addData("turn Right heading beg:", getHeading());
                telemetry.update();
                wheelFrontLeft.setPower(TURNING_SPEED);
                wheelRearLeft.setPower(TURNING_SPEED);
                wheelFrontRight.setPower(-TURNING_SPEED);
                wheelRearRight.setPower(-TURNING_SPEED);
            }
        }

        while(getHeading() > targetAngle) {
            telemetry.addData("turn Right heading beg:", getHeading());
            telemetry.update();
            wheelFrontLeft.setPower(TURNING_SPEED);
            wheelRearLeft.setPower(TURNING_SPEED);
            wheelFrontRight.setPower(-TURNING_SPEED);
            wheelRearRight.setPower(-TURNING_SPEED);
        }

        currentAngle = targetAngle;

//        while ((1 >= heading) || heading >= angle) {      //angle is 270 for 90 deg turn (315 for 45 deg)
//            wheelFrontLeft.setPower(TURNING_SPEED);
//            wheelRearLeft.setPower(TURNING_SPEED);
//            wheelFrontRight.setPower(-TURNING_SPEED);
//            wheelRearRight.setPower(-TURNING_SPEED);
//
//            heading = getHeading();
//            telemetry.addData("turn Right heading:", heading);
//            telemetry.update();
//        }

        wheelFrontLeft.setPower(0);
        wheelRearLeft.setPower(0);
        wheelFrontRight.setPower(0);
        wheelRearRight.setPower(0);

        //requestOpModeStop();
        sleep(250);   // optional pause after each move

    }

    public void turnLeft(int angle) {

        double heading = getHeading();
        double targetAngle = currentAngle + angle;

        telemetry.addData("turn Left heading beg:", heading);
        telemetry.update();

        if(targetAngle >= 360) {
            targetAngle = targetAngle - 360;
            while(getHeading() > 5) {
                telemetry.addData("turn Left heading beg:", getHeading());
                telemetry.update();
                wheelFrontLeft.setPower(-TURNING_SPEED);
                wheelRearLeft.setPower(-TURNING_SPEED);
                wheelFrontRight.setPower(TURNING_SPEED);
                wheelRearRight.setPower(TURNING_SPEED);
            }
        }

        while(getHeading() < targetAngle) {
            telemetry.addData("turn Left heading beg:", getHeading());
            telemetry.update();
            wheelFrontLeft.setPower(-TURNING_SPEED);
            wheelRearLeft.setPower(-TURNING_SPEED);
            wheelFrontRight.setPower(TURNING_SPEED);
            wheelRearRight.setPower(TURNING_SPEED);
        }

        currentAngle = targetAngle;


//        while (heading > 345 || heading <= angle) {         //angle is 90 for 90 deg turns (45 for 45 deg)
//
//            wheelFrontLeft.setPower(-TURNING_SPEED);
//            wheelRearLeft.setPower(-TURNING_SPEED);
//            wheelFrontRight.setPower(TURNING_SPEED);
//            wheelRearRight.setPower(TURNING_SPEED);
//
//            heading = getHeading();
//            telemetry.addData("turn Left heading:", heading);
//            telemetry.update();
//        }

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

    public int getEncoderPulseStrafe(double dist) {
        return (int) ((65.5 * dist) + 27.1 );

    }

    public int getEncoderPulseActuator(double dist) {
        return (int) (dist * ROTATION_PER_IN_LINEAR_ACTUATOR * ENCODER_CPR_LINEAR_ACTUATOR);
    }

    public double getStrafedInchesFromEncoder(double i) {
        return (i * Math.PI * WHEEL_DIA) / (ENCODER_CPR * GEAR_RATIO * Math.sqrt(2));
    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();


        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // parameters.cameraDirection = CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }


    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //  tfodParameters.useObjectTracker = false;
        // tfodParameters.minimumConfidence = 0.75;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);
    }

}

