package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by shivesh on 11/15/2015.
 */
@TeleOp(name="RoverTest:LeftFront",group = "RoverTest")
@Disabled

public class RoverLeftFront extends OpMode{

    private DcMotor leftFront;

    @Override
    public void init() {

        leftFront = hardwareMap.dcMotor.get("wheelFrontLeft");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


    }

    @Override
    public void loop() {
        leftFront.setPower(0.5);
    }

}
