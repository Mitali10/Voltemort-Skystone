package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by shivesh on 11/15/2015.
 */
@TeleOp(name="RoverTest:MotorTest",group = "RoverTest")
@Disabled

public class RoverRightFront extends OpMode{

    private DcMotor frontRight;

    @Override
    public void init() {

        frontRight = hardwareMap.dcMotor.get("wheelFrontRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


    }

    @Override
    public void loop() {
        frontRight.setPower(0.5);
    }

}
