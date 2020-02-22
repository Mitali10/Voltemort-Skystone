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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "SkystoneAutonomousBase", group = "Skystone")
@Disabled
public class SkystoneAutonomousBase extends LinearOpMode {

    enum Task {HALT, MOVE, LOCATE_SKYSTONE, MOVE_TO_SKYSTONE, APPROACH_STONES, SEARCH, LIFT_SKYSTONE, RETURN_TO_STONES, DROP_SKYSTONE2, LIFT_SKYSTONE2, DRIVE_TO_FOUNDATION, DROP_SKYSTONE, MOVE_FOUNDATION, PARK, PARK2}

    Task nextTask;

    DcMotor wheelFrontRight, wheelFrontLeft, wheelRearRight, wheelRearLeft, leftSlide, rightSlide;
    Servo gripperServo, turnServo, foundationServo1, foundationServo2;
    DistanceSensor rangeSensor;

    static final double GRIPPER_CLOSE_POS = .97;
    static final double GRIPPER_OPEN_POS = 0.2;
    static final double ARM_UP = 0.13;
    static final double ARM_DOWN = 0.44;
    static final long DELAY_TIME = 80; // delay between movements

    BNO055IMU imu;
    Orientation angles;
    final double TURNING_SPEED = 0.4;
    final double BACKUP_SPEED = 0.6;

    final double ENCODER_CPR = 383.6; //for goBuilda 435 rmp 753.2; // 188.3; // for goBUILDA 223rpm // last year - 1680;  //1120 for Neverest 40s
    final double GEAR_RATIO = 2;   //changed 1/29
    final double WHEEL_DIA = 3.93;  //in inches
    double currentAngle = 0;
    double distanceTraveled = 0;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    final double ENCODER_CPR_LINEAR_ACTUATOR = 145.6;
    final double ROTATION_PER_IN_LINEAR_ACTUATOR = 3.175;

//
//    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
//    private static final String LABEL_STONE = "Stone";
//    private static final String LABEL_SKYSTONE = "Skystone";
//    private static final String VUFORIA_KEY =
//            "AUMfooL/////AAABmWMP4b7rnE8PhmtEfcgR/5AhEXZ0rcb0BB83YQKUec7YDlux0IMJBg/2VpYxLTwLx0YkvnpKJQeB0mNJVfiBjLaSkcmeUk/TElajJxeeNOwPFTTod0UigGys8lMtKMn8ICHTB/FjOcfF+pZImjOnZ1J5/U60HP/UGWbUYrFyhuusMVHAavfzP3PmR1xTz+Iodh/LnFNrS73ZXosHwmCZQhWh4NxcxxnBS0CwJ79svlkEkZ7Atq6pv+E7Wzt5xQ8pzrU/wiGenHKxquSIx8MIgapxxULbBhHgriKxSctt5sHL8EI6sn8j0nDCPN4TLo6F5/QatwzKURyNVm21yX8qIr5ILFdO8/raVYUxO596aLZt";
//    VuforiaLocalizer vuforia;
//    TFObjectDetector tfod;

    OpenCvCamera cam;
    SamplePipeline pipeline;

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

        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
        turnServo = hardwareMap.get(Servo.class, "turnServo");

        foundationServo1 = hardwareMap.get(Servo.class, "foundationServo1");
        foundationServo2 = hardwareMap.get(Servo.class, "foundationServo2");

        foundationServo1.setPosition(1.0);  // horiz pos
        foundationServo2.setPosition(1.0);  // horiz pos
        turnServo.setPosition(ARM_UP);
        gripperServo.setPosition(GRIPPER_CLOSE_POS);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.useExternalCrystal = true;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");

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
    }

    public void encoderDriveForwardUntilDistance(double speed, double distanceTo, double timeoutS) {

        int encoderStart =  wheelFrontLeft.getCurrentPosition();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            runtime.reset();
            wheelRearLeft.setPower(abs(speed));
            wheelRearRight.setPower(abs(speed));
            wheelFrontLeft.setPower(abs(speed));
            wheelFrontRight.setPower(abs(speed));

            while (opModeIsActive() && (rangeSensor.getDistance(DistanceUnit.INCH) > distanceTo) && (runtime.seconds() < timeoutS)) {
                telemetry.addData("distance: ", rangeSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }

            wheelRearLeft.setPower(0);
            wheelRearRight.setPower(0);
            wheelFrontLeft.setPower(0);
            wheelFrontRight.setPower(0);

            sleep(DELAY_TIME);   // optional pause after each move
        }

        distanceTraveled = getStrafedInchesFromEncoder(  abs(wheelFrontLeft.getCurrentPosition() - encoderStart)  );
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


            sleep(DELAY_TIME);   // optional pause after each move
        }
    }


    public void encoderStrafeLeft(double speed, double distance, double timeoutS) {
        int newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // double strafeDistance = abs(distance) * sqrt(2);
            double strafeDistance = abs(distance);

//            // Determine new target position, and pass to motor controller
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


            sleep(DELAY_TIME);   // optional pause after each move
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


            sleep(DELAY_TIME);   // optional pause after each move
        }
    }

    public void turnRight(int angle) {

        double heading = getHeading();
        // double targetAngle = currentAngle - angle;     //currentAngle is set to 0 always ?
        double targetAngle = getHeading() - angle;

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

        wheelFrontLeft.setPower(0);
        wheelRearLeft.setPower(0);
        wheelFrontRight.setPower(0);
        wheelRearRight.setPower(0);

        //requestOpModeStop();
        sleep(DELAY_TIME);   // optional pause after each move

    }

    public void turnTo90Left() {
        if(getHeading() > 90) {

            while(getHeading() > 92) {
                telemetry.addData("turning right; current heading:", getHeading());
                telemetry.update();
                wheelFrontLeft.setPower(0.1);
                wheelRearLeft.setPower(0.1);
                wheelFrontRight.setPower(-0.1);
                wheelRearRight.setPower(-0.1);
            }


        } else {
            while(getHeading() < 88) {
                telemetry.addData("turning left; current heading:", getHeading());
                telemetry.update();
                wheelFrontLeft.setPower(-0.1);
                wheelRearLeft.setPower(-0.1);
                wheelFrontRight.setPower(0.1);
                wheelRearRight.setPower(0.1);
            }
        }

        wheelFrontLeft.setPower(0);
        wheelRearLeft.setPower(0);
        wheelFrontRight.setPower(0);
        wheelRearRight.setPower(0);

        //requestOpModeStop();
        sleep(DELAY_TIME);   // optional pause after each move

    }


    public void turnTo90Right() {
        if(getHeading() > 270) {

            while(getHeading() > 272) {
                telemetry.addData("turning right; current heading:", getHeading());
                telemetry.update();
                wheelFrontLeft.setPower(0.1);
                wheelRearLeft.setPower(0.1);
                wheelFrontRight.setPower(-0.1);
                wheelRearRight.setPower(-0.1);
            }


        } else {
            while(getHeading() < 268) {
                telemetry.addData("turning left; current heading:", getHeading());
                telemetry.update();
                wheelFrontLeft.setPower(-0.1);
                wheelRearLeft.setPower(-0.1);
                wheelFrontRight.setPower(0.1);
                wheelRearRight.setPower(0.1);
            }
        }

        wheelFrontLeft.setPower(0);
        wheelRearLeft.setPower(0);
        wheelFrontRight.setPower(0);
        wheelRearRight.setPower(0);

        //requestOpModeStop();
        sleep(DELAY_TIME);   // optional pause after each move

    }


    public void turnToZero() {

        if(getHeading() < 180) {

            while(getHeading() > 3) {
                telemetry.addData("turning right; current heading:", getHeading());
                telemetry.update();
                wheelFrontLeft.setPower(0.1);
                wheelRearLeft.setPower(0.1);
                wheelFrontRight.setPower(-0.1);
                wheelRearRight.setPower(-0.1);
            }


        } else {
            while(getHeading() < 357) {
                telemetry.addData("turning left; current heading:", getHeading());
                telemetry.update();
                wheelFrontLeft.setPower(-0.1);
                wheelRearLeft.setPower(-0.1);
                wheelFrontRight.setPower(0.1);
                wheelRearRight.setPower(0.1);
            }
        }

        wheelFrontLeft.setPower(0);
        wheelRearLeft.setPower(0);
        wheelFrontRight.setPower(0);
        wheelRearRight.setPower(0);

        //requestOpModeStop();
        sleep(DELAY_TIME);   // optional pause after each move

    }

    public void turnLeft(int angle) {

        double heading = getHeading();
//        double targetAngle = currentAngle + angle;
        double targetAngle = heading + angle;

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

        wheelFrontLeft.setPower(0);
        wheelRearLeft.setPower(0);
        wheelFrontRight.setPower(0);
        wheelRearRight.setPower(0);

        //requestOpModeStop();
        sleep(DELAY_TIME);   // optional pause after each move

    }

    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle + 360) % 360;

    }

    public int getEncoderPulse(double dist) {
        return (int) ((dist / (Math.PI * WHEEL_DIA)) * ENCODER_CPR * GEAR_RATIO);
    }

    public int getEncoderPulseStrafe(double dist) {
        return (int) ((67 * dist) + 27.1 );

    }

    public double getStrafedInchesFromEncoder(double i) {
        return (i * Math.PI * WHEEL_DIA) / (ENCODER_CPR * GEAR_RATIO * Math.sqrt(2));
    }

    public void initOpenCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        cam.openCameraDevice();

        pipeline = new SamplePipeline();
        cam.setPipeline(pipeline);

        telemetry.addData("Going to start streaming", "");
        telemetry.update();

        cam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    public void gyroAssistedStraightDrive(double speed, double distance,double angle, double timeoutS) {

        int newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget;
        double  max, error, steer ;
        double  leftSpeed, rightSpeed;

        speed = abs(speed);

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
            wheelRearLeft.setPower(speed);
            wheelRearRight.setPower(speed);
            wheelFrontLeft.setPower(speed);
            wheelFrontRight.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (wheelRearLeft.isBusy() && wheelRearRight.isBusy() && wheelFrontLeft.isBusy() && wheelFrontRight.isBusy())) {


                // New code for gyro assist


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                wheelRearLeft.setPower(leftSpeed);
                wheelRearRight.setPower(rightSpeed);
                wheelFrontLeft.setPower(leftSpeed);
                wheelFrontRight.setPower(rightSpeed);


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


    public void gyroAssistedStrafeLeft(double speed, double distance, double angle, double timeoutS) {

        int newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget;
        double  max, error, steer ;
        double  frontSpeed, rearSpeed;

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


                // New code for gyro assist


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                rearSpeed = speed - steer;
                frontSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(rearSpeed), Math.abs(frontSpeed));
                if (max > 1.0)
                {
                    frontSpeed /= max;
                    rearSpeed /= max;
                }

                wheelRearLeft.setPower(rearSpeed);
                wheelRearRight.setPower(rearSpeed);
                wheelFrontLeft.setPower(frontSpeed);
                wheelFrontRight.setPower(frontSpeed);

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

    public void gyroAssistedStrafeRight(double speed, double distance, double angle, double timeoutS) {

        int newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget;
        double  max, error, steer ;
        double  frontSpeed, rearSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // double strafeDistance = abs(distance) * sqrt(2);
            double strafeDistance = abs(distance);

            // Determine new target position, and pass to motor controller
//            newRearLeftTarget = wheelRearLeft.getCurrentPosition() + getEncoderPulse(strafeDistance);
//            newRearRightTarget = wheelRearRight.getCurrentPosition() - getEncoderPulse(strafeDistance);
//            newFrontLeftTarget = wheelFrontLeft.getCurrentPosition() - getEncoderPulse(strafeDistance);
//            newFrontRightTarget = wheelFrontRight.getCurrentPosition() + getEncoderPulse(strafeDistance);

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


                // New code for gyro assist


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                rearSpeed = speed + steer;
                frontSpeed = speed - steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(rearSpeed), Math.abs(frontSpeed));
                if (max > 1.0)
                {
                    frontSpeed /= max;
                    rearSpeed /= max;
                }

                wheelRearLeft.setPower(rearSpeed);
                wheelRearRight.setPower(rearSpeed);
                wheelFrontLeft.setPower(frontSpeed);
                wheelFrontRight.setPower(frontSpeed);

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


    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.

        wheelRearLeft.setPower(leftSpeed);
        wheelRearRight.setPower(rightSpeed);
        wheelFrontLeft.setPower(leftSpeed);
        wheelFrontRight.setPower(rightSpeed);

        return onTarget;
    }

}

