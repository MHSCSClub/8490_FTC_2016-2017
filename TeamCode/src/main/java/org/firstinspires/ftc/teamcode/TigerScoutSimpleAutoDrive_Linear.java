package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *
 * The code REQUIRES encoders
 *
 *   The desired program is:
 *   - Fire 1 ball
 *   - Load a second ball
 *   - Fire the other ball
 *   - Move forward 67 inches (onto the base, knocking off the cap ball)
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement, based off of the AutoDriveByEncoder Example
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  Separate methods control the popper (shooter) and the pickup mechanism
 */

@Autonomous(name="TigerScout: Auto Drive (Simple, Universal)", group="TigerScout")
public class TigerScoutSimpleAutoDrive_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTigerScout      robot   = new HardwareTigerScout();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = Motors.ANDYMARK_40_CPR;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.5;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flipper.setPosition(0);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.frontLeftMotor.getCurrentPosition(),
                          robot.frontRightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Waiting", "Will wait 12 seconds!!");

        sleep(12 * 1000);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        popper(3); //Fire 1 ball
        pickup(1.3); //Load next ball
        popper(3); //Fire next ball
        encoderDrive(DRIVE_SPEED,  67,  67, 8.0); //Drive to central vortex assembly

        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTargetA;
        int newLeftTargetB;
        int newRightTargetA;
        int newRightTargetB;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTargetA = robot.frontLeftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftTargetB = robot.backLeftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTargetA = robot.frontRightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightTargetB = robot.backRightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.frontLeftMotor.setTargetPosition(newLeftTargetA);
            robot.backLeftMotor.setTargetPosition(newLeftTargetB);
            robot.frontRightMotor.setTargetPosition(newRightTargetA);
            robot.backRightMotor.setTargetPosition(newRightTargetB);

            // Turn On RUN_TO_POSITION
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeftMotor.setPower(Math.abs(speed));
            robot.backLeftMotor.setPower(Math.abs(speed));
            robot.frontRightMotor.setPower(Math.abs(speed));
            robot.backRightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and all motors are running.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() &&
                           robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTargetA,  newRightTargetB);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.frontLeftMotor.getCurrentPosition(),
                                            robot.backLeftMotor.getCurrentPosition(),
                                            robot.frontRightMotor.getCurrentPosition(),
                                            robot.backRightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeftMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    private void popper(double timeoutS){
        robot.popper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.popper.setTargetPosition(robot.popper.getCurrentPosition()
                - HardwareTigerScout.POPPER_CPR);
        robot.popper.setPower(-1);

        while(opModeIsActive() && runtime.seconds() < timeoutS && robot.popper.isBusy()) {
            telemetry.addData("Popper", "%.2f", (float) robot.popper.getCurrentPosition());
            telemetry.update();
            sleep(50);
        }
    }
    private void pickup(double timeoutS){
        runtime.reset();
        double power = 0.6;
        robot.pickup.setPower(power);
        telemetry.addData("pickup", "%.2f", power);
        while(opModeIsActive() && runtime.seconds() < timeoutS);
        power = 0;
        robot.pickup.setPower(power);
        telemetry.addData("pickup", "%.2f", power);
    }
}
