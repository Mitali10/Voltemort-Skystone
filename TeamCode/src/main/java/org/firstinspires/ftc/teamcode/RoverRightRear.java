package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by shivesh on 11/15/2015.
 */
@TeleOp(name="RoverTest:RightRear",group = "RoverTest")
@Disabled

public class RoverRightRear extends OpMode{

    private DcMotor rightRear;

    @Override
    public void init() {

        rightRear = hardwareMap.dcMotor.get("wheelRearRight");

        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


    }

    @Override
    public void loop() {
        rightRear.setPower(0.5);
    }

}
