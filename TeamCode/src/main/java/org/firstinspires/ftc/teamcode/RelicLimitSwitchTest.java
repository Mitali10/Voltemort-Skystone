package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by shivesh on 11/15/2015.
 */
@TeleOp(name="RelicTest:RelicLimitSwitchTest",group = "RelicTest")
@Disabled

public class RelicLimitSwitchTest extends OpMode{

    private DigitalChannel limitSwitch;
	

    @Override
    public void init() {

	limitSwitch = hardwareMap.get(DigitalChannel.class, "limit");
   
    }

    @Override
    public void loop() {
        telemetry.addData("Switch State ", limitSwitch.getState());
    }

}
