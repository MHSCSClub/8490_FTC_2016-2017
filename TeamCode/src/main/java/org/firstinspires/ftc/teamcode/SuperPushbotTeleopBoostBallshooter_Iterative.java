package org.firstinspires.ftc.teamcode;/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for Group 1's robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the org.firstinspires.ftc.teamcode.Pushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for Group 1's robot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Disabled
@TeleOp(name="SuperPushbot: Teleop + Ball Shooter (Jack and Yi's Boost)", group="SuperPushbot")
public class SuperPushbotTeleopBoostBallshooter_Iterative extends OpMode{

    //Constants
    private static final float BALLSHOOTER_POWER = 0.25f;

    /* Declare OpMode members. */
    SuperPushbot robot = new SuperPushbot();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //Send a message to signify init worked
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        telemetry.addData("Say", "Started robot!");    //Send a message to signify run started
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        leftBoostMovement();
        activateBallshooter();
        // Send telemetry message to signify robot running;

        updateTelemetry(telemetry);
    }

    private void activateBallshooter(){
        float power = gamepad1.left_bumper ? BALLSHOOTER_POWER : 0;
        robot.pitchRight.setPower(power);
        robot.pitchLeft.setPower(power);
        telemetry.addData("ballshooter", "%.2f", power);
    }



    /**
     * Jack and Yi's proprietary movement method (modified for the new robot)
     */
    private void leftBoostMovement() {
        float right = gamepad1.left_stick_y - gamepad1.left_stick_x;
        float left = gamepad1.left_stick_y +  gamepad1.left_stick_x;


        //Scale inputs
        right = (float)scaleInput(right) / 2f;
        left =  (float)scaleInput(left) / 2f;

        float boost = gamepad1.left_trigger;
        boost = boost < .2f ? .2f : boost; //min value of boost is .2
        boost = (float) scaleInput(boost) * 2f;

        //add boost amount to vectors right and left
        right *= boost;
        left *= boost;

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        robot.frontRightMotor.setPower(left);
        robot.backRightMotor.setPower(left);

        robot.frontLeftMotor.setPower(right);
        robot.backLeftMotor.setPower(right);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);


    }

    /*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
    private double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.pitchLeft.setPower(0);
        robot.pitchRight.setPower(0);
        telemetry.addData("Say", "Stopped robot!");    //Send a message to signify run stopped
        updateTelemetry(telemetry);
    }

}
