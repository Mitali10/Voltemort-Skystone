/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name="Auto:GyroRightTurn",group = "Auto")
@Disabled
public class RelicGyroRightTurn extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor wheelRight,wheelLeft;
  private GyroSensor gyro;
  private ModernRoboticsI2cGyro MRgyro;
  private IntegratingGyroscope Intgyro;
  
  private final double TURNING_SPEED =0.25;
  private final double turnRight90Angle =-90 ;
  private final double turnLeft90Angle=90;


  @Override
  public void init() {

    wheelLeft = hardwareMap.get(DcMotor.class,"motorLeft");
    wheelRight = hardwareMap.get(DcMotor.class,"motorRight");

    wheelLeft.setDirection(DcMotor.Direction.REVERSE);
    wheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    wheelRight.setDirection(DcMotor.Direction.FORWARD);
    wheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	
	/*
    gyro = hardwareMap.get(GyroSensor.class,"gyro");
	gyro.calibrate();
	gyro.resetZAxisIntegrator();
	*/
	
	MRgyro = hardwareMap.get(ModernRoboticsI2cGyro.class,"gyro");
    MRgyro.calibrate();
	MRgyro.resetZAxisIntegrator();

  }



  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
	  
	  
	
	if (MRgyro.getIntegratedZValue()>=turnRight90Angle){


      telemetry.addData("Right now at ",MRgyro.getIntegratedZValue());

      // frontLeft is wheel left
      // wheelLeft is front left

      wheelLeft.setPower(TURNING_SPEED);
      wheelRight.setPower(-TURNING_SPEED);



    } else {

      wheelRight.setPower(0.0);
      wheelLeft.setPower(0.0);
      telemetry.addData("Turning Completed", MRgyro.getIntegratedZValue());
    }
  }	
	  
	  
  /*
  
    if (Math.abs(turnRight90Angle-gyro.getHeading())>1){


      telemetry.addData("Right now at ",gyro.getHeading());

      // frontLeft is wheel left
      // wheelLeft is front left

      wheelLeft.setPower(TURNING_SPEED);
      wheelRight.setPower(-TURNING_SPEED);



    } else {

      wheelRight.setPower(0.0);
      wheelLeft.setPower(0.0);
      telemetry.addData("Turning Completed", gyro.getHeading());
      requestOpModeStop();
    }
  }

*/


}
