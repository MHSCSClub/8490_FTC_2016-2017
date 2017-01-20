package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Autonomous for BLUE. For RED, we turn 57 degrees to align with beacon, and we look for red
 * instead of blue on the left beacon.
 * The code REQUIRES encoders, an MR Gyro, and an MR Color Sensor.
 *
 *   The desired program is:
 *   - Fire 1 ball
 *   - Load a second ball
 *   - Fire the other ball
 *   - Move forward 67.5 inches (onto the base, knocking off the cap ball)
 *   - Turn -57 degrees
 *   - Move forward 60 inches (to beacon)
 *   - Detect if left side is correct color
 *   - Press appropriate beacon button
 *   - Move backward 60 inches (to park on central vortex again)
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement, based off of the AutoDriveByEncoder Example
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * The code also uses the gyroDrive, gyroTurn, and gyroHold methods, modified from the PushbotAutoDriveByGyro
 * example.
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  Separate methods control the popper (shooter) and the pickup mechanism
 */


public abstract class AutonomousPaulyP123 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTigerScout         robot   = new HardwareTigerScout();   // Use a Pushbot's hardware


    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 1;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    private ElapsedTime     runtime = new ElapsedTime();

    private int turn_direction = redOrBlue() ? 1 : -1;

    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        robot.gyro.calibrate();


        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && robot.gyro.isCalibrating())  {
            sleep(50);
            idle();
        }
        robot.gyro.resetZAxisIntegrator();

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }
        robot.gyro.resetZAxisIntegrator();
        if(!opModeIsActive()){
            stop();
            return;
        }
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turnBlueAutonomousPaulyP123

        flipper(0);
        popper(3); //Fire 1 ball
        pickup(2); //Load next ball (Consider decreasing this value after zip ties implemented?)
        popper(3); //Fire next ball
        flipper(1);

        encoderDrive(DRIVE_SPEED, -66, -66, 8);    // Drive FWD 67.5 inches
        gyroTurn( TURN_SPEED, turn_direction * 47);         // Turn  CW to 57 Degrees
        gyroHold( TURN_SPEED, turn_direction * 47, 0.5);    // Hold 57 Deg heading for a 1/2 second
        encoderDrive(DRIVE_SPEED,-46, -46, 8);    // Drive Forward 60 inches

        //DO the beacons now
        switch(beaconData(!redOrBlue())){  //Check if blue is on left
            case -1:
                //Blue is on left. Poke it!
                flipper(0);
                encoderDrive(DRIVE_SPEED,-8, -8, 8);    // Drive Forward 60 inches

                encoderDrive(DRIVE_SPEED, 52, 52, 8);
                break;
            case 1:
                flipper(2);
                encoderDrive(DRIVE_SPEED,-8, -8, 8);    // Drive Forward 60 inches

                encoderDrive(DRIVE_SPEED, 52, 52, 8);
                break;
            default:
                encoderDrive(DRIVE_SPEED, 59, 59, 8);
        };

            // Drive REV 60 inches, to center base



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /**
     *
     * @param lookForBlue True: blue, False: red
     * @return -1 for left, 1 for right, 0 for wtf
     */
    public int beaconData(boolean lookForBlue){
        float hsvValues[] = {0F,0F,0F};
        robot.colorSensor.enableLed(false);
        Color.RGBToHSV(robot.colorSensor.red() * 8, robot.colorSensor.green() * 8, robot.colorSensor.blue() * 8, hsvValues);
        float hue = hsvValues[0];
        telemetry.addData("Hue ", hue);
        boolean isRed;
        if(hue < 10 || (hue > 310 && hue < 350)){
            //RED
            telemetry.addData("Color", "Probably red");
            telemetry.update();
            return lookForBlue ? -1 : 1;
        } else if(hue > 200 && hue < 250){
            //BLUE
            telemetry.addData("Color", "Probably Blue");
            telemetry.update();
            return lookForBlue ? 1 : -1;
        } else {
            telemetry.addData("Color", "Unknown");
            telemetry.update();
            return 0;
        }
    }

   
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return if is currently on required heading
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;


        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.frontLeftMotor.setPower(-leftSpeed);
        robot.backLeftMotor.setPower(-leftSpeed);
        robot.frontRightMotor.setPower(-rightSpeed);
        robot.backRightMotor.setPower(-rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    private void popper(double timeoutS){
        robot.popper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.popper.setTargetPosition(robot.popper.getCurrentPosition()
                - (int)HardwareTigerScout.POPPER_CPR);
        robot.popper.setPower(-1);

        while(opModeIsActive() && runtime.seconds() < timeoutS && robot.popper.isBusy()) {
            telemetry.addData("Popper", "%.2f", (float) robot.popper.getCurrentPosition());
            telemetry.update();
            sleep(50);
        }
    }
    private void pickup(double timeoutS){
        runtime.reset();
        robot.pickup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(50);
        robot.pickup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double power = 0.6;
        robot.pickup.setPower(power);
        robot.pickup.setTargetPosition(robot.pickup.getCurrentPosition() + (int)HardwareTigerScout.PICKUP_CPR);
        telemetry.addData("pickup", "%.2f", power);
        while(opModeIsActive() && runtime.seconds() < timeoutS){
            idle();
            telemetry.addData("pickup_pos", "%.2f", robot.pickup.getCurrentPosition());
        }
        power = 0;
        robot.pickup.setPower(power);
        telemetry.addData("pickup", "%.2f", power);
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
            newLeftTargetA = robot.frontLeftMotor.getCurrentPosition() + (int)(leftInches * HardwareTigerScout.DRIVE_COUNTS_PER_INCH);
            newLeftTargetB = robot.backLeftMotor.getCurrentPosition() + (int)(leftInches * HardwareTigerScout.DRIVE_COUNTS_PER_INCH);
            newRightTargetA = robot.frontRightMotor.getCurrentPosition() + (int)(rightInches * HardwareTigerScout.DRIVE_COUNTS_PER_INCH);
            newRightTargetB = robot.backRightMotor.getCurrentPosition() + (int)(rightInches * HardwareTigerScout.DRIVE_COUNTS_PER_INCH);
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


    /**
     *
     * @param flipper_state 0 is left, 1 is mid, 2 is right
     */
    private void flipper(int flipper_state){
        double flipperPos;
        switch(flipper_state){
            case 0:
                flipperPos = 0;
                break;
            case 1:
                flipperPos = 0.5;
                break;
            case 2:
                flipperPos = 1;
                break;
            default:
                flipperPos = -1;
        }
        robot.flipper.setPosition(flipperPos);

    }

    public abstract boolean redOrBlue();
}
