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


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Red_Right", group="Pushbot")
// @Disabled
public class Red_Right extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.5;
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/target_v2_meta.tflite";
    private static final String[] LABELS = {
      "Target",
      //"Blue",
      //"Duck",
      //"Marker"
    };
    private static final String VUFORIA_KEY =
            "ATN3j0H/////AAABmXMHTmx3bEG+tSTXv+YZJlhXqx7uZ5UL4z6V0HKRkncSJFv/AWWcovnsXC+V5wu8HV1XGHh48YX1TrEBq5TQOcEOn2DaTAMVLkn6SHulDN2WTKs3CnKC6ftpso4OC7tI2e7gVZiLFC19Hi/i/T3GsxpPCrSq7U5ArjefnEa3IVblTwb+D+Eo4fAzS7ymJnbELQ1w7kMzWcqnx/2J+4G1sHf2ooxdKU3CM2twP53H5fkc7xgkyw3eRCXRhy0/Kb/cAnHTmeZDq863Amit6EX7uh79sSGuUZonYoIkmi/5e/9uDPLa7GXgAO1ntvM+NxnEp8G3oHUUZrm9NJdfCpp7BbvqcwbYWIHroj1YmmeJDVwp";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    public int pos;
    
    
    

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.3, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        boolean whileFlag = true;
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      // step through the list of recognitions and display boundary info.
                      int i = 0;
                      if (whileFlag == false) break;
                      for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                        float height = recognition.getBottom() - recognition.getTop();
                        if(height > 130 && height < 200){
                            //check position
                            if(recognition.getRight()> 600){
                                pos = 2;
                            }else if(recognition.getRight()> 400){
                                pos = 1;
                            }else{
                                pos = 0;
                            }
                            telemetry.addData(String.format("position (%d)", pos), recognition.getLabel());
                            whileFlag = false;
                            break; 
                        }
                        
                        
                      }
                      
                      telemetry.update();
                    }
                }
            }
        } 
        
        robot.init(hardwareMap);
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
        // Arm
        robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.frontLeftDrive.getCurrentPosition(),
                          robot.frontRightDrive.getCurrentPosition());
        telemetry.update();

 

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        
        encoderDrive(DRIVE_SPEED,  36, 36, 36, 36, 5.0); 
        
        encoderDrive(TURN_SPEED,   17.27, -17.27, 17.27, -17.27, 5.0);
        //encoderDrive(DRIVE_SPEED,  -5.7, -5.7, -5.7, -5.7, 5.0);
        
        //1st level
        if(pos == 0){
            encoderDrive(DRIVE_SPEED,  -5, -5, -5, -5, 5.0);
            
            robot.elevator.setTargetPosition(1800);
            robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.elevator.setPower(1);
                
            sleep(1000);
            robot.box.setPosition(0.6);
                
            sleep(1000);
            robot.elevator.setTargetPosition(0);
            robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.elevator.setPower(0.7);
                
            sleep(1000);
            robot.box.setPosition(0.77);
            sleep(1000);
                
            robot.elevator.setTargetPosition(1800);
            robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.elevator.setPower(1);
                
            sleep(1000);
            robot.box.setPosition(0.02);
            sleep(1000);}
        
        //2nd level    
        if(pos == 1){
            encoderDrive(DRIVE_SPEED,  -5.9, -5.9, -5.9, -5.9, 5.0);
            
            robot.elevator.setTargetPosition(1800);
            robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.elevator.setPower(1);
                
            sleep(1000);
            robot.box.setPosition(0.5);
                
            sleep(1000);
            robot.elevator.setTargetPosition(800);
            robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.elevator.setPower(0.7);
                
            sleep(1000);
            robot.box.setPosition(0.77);
            sleep(1000);
                
            robot.elevator.setTargetPosition(1800);
            robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.elevator.setPower(1);
                
            sleep(1000);
            robot.box.setPosition(0.02);
            sleep(1000);}
        
        //3rd level    
        if(pos == 2){
            encoderDrive(DRIVE_SPEED,  -6, -6, -6, -6, 5.0);
            
            robot.elevator.setTargetPosition(2300);
            robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.elevator.setPower(1);
                
            sleep(1000);
            robot.box.setPosition(0.8);
            sleep(2000);
            robot.box.setPosition(0.02);
            sleep(1000);}
        
        
        encoderDrive(DRIVE_SPEED,  20, -20, -20, 20, 5.0);
        encoderDrive(1.0,  55, 55, 55, 55, 5.0);
        
robot.elevator.setTargetPosition(0);
robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
robot.elevator.setPower(-1);

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
                             double frontleftInches, double frontrightInches,
                             double backleftInches, double backrightInches,
                             double timeoutS) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = robot.frontLeftDrive.getCurrentPosition() - (int)(frontleftInches * COUNTS_PER_INCH);
            newfrontRightTarget = robot.frontRightDrive.getCurrentPosition() + (int)(frontrightInches * COUNTS_PER_INCH);
            newbackLeftTarget = robot.backLeftDrive.getCurrentPosition() + (int)(backleftInches * COUNTS_PER_INCH);
            newbackRightTarget = robot.backRightDrive.getCurrentPosition() - (int)(backrightInches * COUNTS_PER_INCH);
            
            robot.frontLeftDrive.setTargetPosition(newfrontLeftTarget);
            robot.frontRightDrive.setTargetPosition(newfrontRightTarget);
            robot.backLeftDrive.setTargetPosition(newbackLeftTarget);
            robot.backRightDrive.setTargetPosition(newbackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeftDrive.setPower(Math.abs(speed));
            robot.frontRightDrive.setPower(Math.abs(speed));
            robot.backLeftDrive.setPower(Math.abs(speed));
            robot.backRightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newfrontLeftTarget,  newfrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.frontLeftDrive.getCurrentPosition(),
                                            robot.frontRightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderArm(double speed,
                             double Inches,
                             double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = robot.elevator.getCurrentPosition() - (int)(Inches * COUNTS_PER_INCH);
            
            robot.elevator.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.elevator.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.elevator.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", newTarget);
                telemetry.addData("Path2",  "Running at %7d",
                                            robot.elevator.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.elevator.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.95f;
       tfodParameters.isModelTensorFlow2 = true;
       tfodParameters.inputSize = 320;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
       tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
}
