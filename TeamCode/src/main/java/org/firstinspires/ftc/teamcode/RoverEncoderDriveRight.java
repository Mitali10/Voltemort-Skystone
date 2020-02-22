package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

@Autonomous(name="RoverTest:RoverEncoderDriveRight",group = "RoverTest")
@Disabled
public class RoverEncoderDriveRight extends LinearOpMode{

    enum Task{RUN_STRAIGHT, HALT };

    org.firstinspires.ftc.teamcode.RoverEncoderDrive.Task nextTask;

    private DcMotor wheelFrontRight , wheelFrontLeft, wheelRearRight , wheelRearLeft;

    private final double RUN_STRAIGHT_SPEED = 0.6;
    private final double TURNING_SPEED =0.2;
    private final double BACKUP_SPEED =0.6;


    private final int ENCODER_CPR = 1680;  //1120 for Neverest 40s
    private  final double GEAR_RATIO = 1;   //changed 1/29
    private final int WHEEL_DIA = 6;  //in inches

    private ElapsedTime runtime = new ElapsedTime();




    @Override
    public void runOpMode() {

        wheelFrontRight = hardwareMap.get(DcMotor.class,"wheelFrontRight");
        wheelFrontLeft = hardwareMap.get(DcMotor.class,"wheelFrontLeft");
        wheelRearRight = hardwareMap.get(DcMotor.class,"wheelRearRight");
        wheelRearLeft = hardwareMap.get(DcMotor.class,"wheelRearLeft");


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





        nextTask = org.firstinspires.ftc.teamcode.RoverEncoderDrive.Task.RUN_STRAIGHT;

        waitForStart();


        while (opModeIsActive()) {


            switch (nextTask) {

                case RUN_STRAIGHT:

                    encoderStrafeRight(0.2,70,8);
                    nextTask = org.firstinspires.ftc.teamcode.RoverEncoderDrive.Task.HALT;

                    break;


                case HALT:

                    break;

            }


        }

        // idle();

    }


    public void encoderDrive(double speed, double distance, double timeoutS)
    {
        int newFrontLeftTarget, newFrontRightTarget,newRearLeftTarget, newRearRightTarget;

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


    public void encoderStrafeLeft(double speed, double distance, double timeoutS)
    {
        int newFrontLeftTarget, newFrontRightTarget,newRearLeftTarget, newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            double strafeDistance = abs(distance)*sqrt(2);
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

    public void encoderStrafeRight(double speed, double distance, double timeoutS)
    {
        int newFrontLeftTarget, newFrontRightTarget,newRearLeftTarget, newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            double strafeDistance = abs(distance)/sqrt(2);
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


    private int getEncoderPulse(double dist){

        return (int) ((dist / (Math.PI * WHEEL_DIA)) * ENCODER_CPR * GEAR_RATIO);
    }
}



