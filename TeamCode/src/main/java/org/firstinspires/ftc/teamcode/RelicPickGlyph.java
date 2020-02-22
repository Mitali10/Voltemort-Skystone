package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by shivesh on 11/15/2015.
 */
@Autonomous(name="Auto:PickGlyph",group = "Auto")
@Disabled
public class RelicPickGlyph extends OpMode {

    private DcMotor motorLeft, motorRight, towerMotor;
    private Servo gripperLeft, gripperRight;
    private double towerMotorPower = 0.3;
    private boolean timePassed;


    @Override
    public void init() {

        towerMotor = hardwareMap.get(DcMotor.class, "towerMotor");

        towerMotor.setDirection(DcMotor.Direction.FORWARD);
        towerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gripperLeft = hardwareMap.get(Servo.class, "gripperLeft");
        gripperLeft.setPosition(0.05);

        gripperRight = hardwareMap.get(Servo.class, "gripperRight");
        gripperRight.setPosition(0.90);

    }

    @Override
    public void start() {
        resetStartTime();

    }

    @Override
    public void loop() {

        gripperLeft.setPosition(0.65);  //changed from 0.6
        gripperRight.setPosition(0.25); //changed from 0.3

        if (time >1 && time < 1.5) {
                towerMotor.setPower(0.3);
            } else {
                towerMotor.setPower(0);
            }


    }
}
