package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

@Autonomous(name="RoverTest:strafeLefting",group = "RoverTest")
@Disabled
public class strafeLefting extends LinearOpMode{

    enum Task{STRAFE_LEFT, HALT };

    Task nextTask;

    private DcMotor wheelFrontRight , wheelFrontLeft, wheelRearRight , wheelRearLeft;
    private final double STRAFE_SPEED = 0.2;

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

        wheelFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRearLeft.setDirection(DcMotor.Direction.REVERSE);
        wheelRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheelFrontRight.setDirection(DcMotor.Direction.FORWARD);
        wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRearRight.setDirection(DcMotor.Direction.FORWARD);
        wheelRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        nextTask = Task.STRAFE_LEFT;

        waitForStart();
        runtime.reset();
        telemetry.addData("nextTask:", nextTask);
        telemetry.update();


        while (opModeIsActive()) {


            switch (nextTask) {

                case STRAFE_LEFT:

                    wheelRearLeft.setPower(-STRAFE_SPEED); //+
                    wheelRearRight.setPower(STRAFE_SPEED); //-
                    wheelFrontLeft.setPower(STRAFE_SPEED); //-
                    wheelFrontRight.setPower(-STRAFE_SPEED); //+

                    nextTask = Task.HALT;

                    telemetry.addData("nextTask:", nextTask);
                    telemetry.update();


                case HALT:

                    if (runtime.seconds() > 8) {
                        wheelRearLeft.setPower(0);
                        wheelRearRight.setPower(0);
                        wheelFrontLeft.setPower(0);
                        wheelFrontRight.setPower(0);
                        telemetry.addData("Run out of Time:", 8);
                        telemetry.update();
                    }


                    break;

            }

        }

    }

}



