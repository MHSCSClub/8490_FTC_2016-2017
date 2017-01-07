package org.firstinspires.ftc.teamcode;

/*
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for Group 1's robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common HardwareTigerScout hardware class to define the devices on the robot.
 * All device access is managed through the org.firstinspires.ftc.teamcode.HardwareTigerScout class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for Group 1's robot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TigerScout: Teleop Boost - Single Controller", group="Tiger Scout")
public class TigerScoutTeleopBoost1Controller_Iterative extends OpMode{

    private boolean last_lbump_state = false;
    private boolean flipper_state = true;
    private boolean dpad_used = false;
    private boolean boost_enabled;

    //Constants

    /* Declare OpMode members. */
    HardwareTigerScout robot = new HardwareTigerScout();

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


        //Disable encoders for teleop, for more speed
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        if(!dpad_used) {
            leftBoostMovement();
        }
        pickup();
        popper();
        flipper();
        dpad();
        // Send telemetry message to signify robot running;
        updateTelemetry(telemetry);
    }

    private void dpad(){
        if(gamepad1.dpad_up || gamepad1.dpad_down || dpad_used){
            dpad_used = gamepad1.dpad_up || gamepad1.dpad_down;
            double power = 0;
            if(gamepad1.dpad_up){
                power = -0.3;
            } else if(gamepad1.dpad_down){
                power = 0.3;
            }
            /*
            if(power != 0){
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }*/
            robot.frontRightMotor.setPower(power);
            robot.backRightMotor.setPower(power);
            robot.frontLeftMotor.setPower(power);
            robot.backLeftMotor.setPower(power);
        }
    }

    private void flipper(){
        if(gamepad1.left_bumper != last_lbump_state){
            last_lbump_state = gamepad1.left_bumper;
            if (gamepad1.left_bumper){
                flipper_state = !flipper_state;
            }
        }
        robot.flipper.setPosition(flipper_state ? 0 : 1);
    }

    private void popper(){
            if (gamepad1.x) {
                if(!robot.popper.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER)) {
                    robot.popper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                robot.popper.setPower(-scaleInput(gamepad1.right_stick_y));
            } else {
                if(robot.popper.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER)){
                    robot.popper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.popper.setTargetPosition(robot.popper.getCurrentPosition());
                }
                if(gamepad1.y){
                    robot.popper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.popper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.popper.setTargetPosition(robot.popper.getCurrentPosition());
                    robot.popper.setPower(0);
                }
                boolean motorStopped = robot.popper.getTargetPosition() >= robot.popper.getCurrentPosition() - 1
                        && robot.popper.getTargetPosition() <= robot.popper.getTargetPosition() + 1;

                if(gamepad1.right_bumper && motorStopped){
                    robot.popper.setTargetPosition(robot.popper.getCurrentPosition() - HardwareTigerScout.POPPER_CPR);
                    robot.popper.setPower(-1);
                } else if(motorStopped){
                    robot.popper.setPower(0);
                }
            }


        telemetry.addData("Popper", "%.2f", (float)robot.popper.getCurrentPosition());
    }

    private void pickup(){
        double power;
        if(gamepad1.a){
            power = 1;
        } else if(gamepad1.b){
            power = -1;
        } else {
            power = 0;
        }
        robot.pickup.setPower(power);
        telemetry.addData("pickup",  "%.2f", power);
    }


    private void leftBoostMovement() {
        float right = gamepad1.left_stick_y - gamepad1.left_stick_x;
        float left = gamepad1.left_stick_y +  gamepad1.left_stick_x;

        right /=10f;
        left /= 10f;

        telemetry.addData("boost","%.2f",gamepad1.left_trigger);
        telemetry.addData("boost-scaled","%.2f",(float)scaleInput(gamepad1.left_trigger));
        if(gamepad1.left_trigger > 0.2){
            if(!boost_enabled){
                robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            boost_enabled = true;
            right *= 10f * (float)scaleInput(gamepad1.left_trigger);
            left *= 10f * (float)scaleInput(gamepad1.left_trigger);
            resetStartTime();
        } else if (boost_enabled && getRuntime() > 2){
            boost_enabled = false;
            robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

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
        telemetry.addData("Say", "Stopped robot!");    //Send a message to signify run stopped
        updateTelemetry(telemetry);
    }

}
