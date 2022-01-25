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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="motor", group="Pushbot")
// @Disabled
public class MecanumDrive extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    double          clawOffset  = 0.02 ;                  // Servo mid position
    int          ARM_POWER   = 0;
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    @Override
    public void runOpMode() {
        double x1;
        double y1;
        
        double xc;
        double yc;
        
        double cw;
        double aw;
        double curr_pos;
        
        double fortyFiveInRads = -Math.PI/4;
        double cosine45 = Math.cos(fortyFiveInRads);
        double sine45 = Math.sin(fortyFiveInRads);


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
        
        robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d",
                          robot.elevator.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            
            // if (gamepad1.right_stick_x < 0){
            //     robot.frontLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            //     robot.frontRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            //     robot.backLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            //     robot.backRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            //     robot.frontLeftDrive.setPower(gamepad1.right_stick_y/2);
            //     robot.frontRightDrive.setPower(gamepad1.right_stick_y/2);
            //     robot.backRightDrive.setPower(gamepad1.right_stick_y/2);
            //     robot.backLeftDrive.setPower(gamepad1.right_stick_y/2);}
            if (gamepad1.right_stick_x != 0){
                robot.frontLeftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
                robot.frontRightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
                robot.backLeftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
                robot.backRightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
                robot.frontLeftDrive.setPower(gamepad1.right_stick_x/2);
                robot.frontRightDrive.setPower(gamepad1.right_stick_x/2);
                robot.backRightDrive.setPower(gamepad1.right_stick_x/2);
                robot.backLeftDrive.setPower(gamepad1.right_stick_x/2);}
            else{
                robot.frontLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
                robot.frontRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
                robot.backLeftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
                robot.backRightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
                            
                y1 = -gamepad1.left_stick_x;
                x1  =  -gamepad1.left_stick_y;
          
                
                // need to rotate 45 degrees.
                yc = y1*cosine45 + x1*sine45;
                xc = x1*cosine45 - y1*sine45;
    
    
                // Output the safe vales to the motor drives.
                robot.frontLeftDrive.setPower(yc);
                robot.backRightDrive.setPower(yc);     
                
                robot.frontRightDrive.setPower(xc);
                robot.backLeftDrive.setPower(xc);
                }
 
            
            
            
            // Use gamepad left & right Bumpers to control hat
            if (gamepad1.dpad_down)
                clawOffset += CLAW_SPEED;
            else if (gamepad1.dpad_up)
                clawOffset -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, 0.02, 0.92);
            robot.hat.setPosition(robot.MID_SERVO + clawOffset);
            
            
            // Use gamepad buttons to move the intake up (Y) and down (A)
            if (gamepad1.y && ARM_POWER == 1){
                ARM_POWER = 0;
                robot.intake.setPower(0);}
            else if (gamepad1.y && ARM_POWER == 0){
                ARM_POWER = 1;
                robot.intake.setPower(1);}
            else
                robot.intake.setPower(ARM_POWER);
            
            //robot.leftArm.setPower(ARM_POWER);
            
            // Use gamepad buttons to spin
            if (gamepad1.left_bumper)
                robot.duck.setPower(1);
            else if (gamepad1.right_bumper)
                robot.duck.setPower(-1);
            else
                robot.duck.setPower(0.0);
                
            // 3rd level
            if (gamepad1.x){
                curr_pos = robot.elevator.getCurrentPosition();
                
                telemetry.addData("curr_pos", curr_pos);
                telemetry.update();    
                
                robot.elevator.setTargetPosition(2300);
                robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.elevator.setPower(1);
                
                sleep(1000);
                robot.box.setPosition(0.8);
                sleep(2000);
                robot.box.setPosition(0.02);
                sleep(1000);
                
                // robot.elevator.setTargetPosition(0);
                // robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // robot.elevator.setPower(0.7);
            }
            // Put down
            else if (gamepad1.a){
                curr_pos = robot.elevator.getCurrentPosition();
                
                telemetry.addData("curr_pos", curr_pos);
                telemetry.update();    
                
                // robot.elevator.setTargetPosition(1600);
                // robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // robot.elevator.setPower(1);
                
                // sleep(1000);
                // robot.box.setPosition(0.5);
                
                // sleep(1000);
                // robot.elevator.setTargetPosition(1000);
                // robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // robot.elevator.setPower(0.7);
                
                // sleep(1000);
                // robot.box.setPosition(0.77);
                // sleep(1000);
                
                // robot.elevator.setTargetPosition(1600);
                // robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // robot.elevator.setPower(1);
                
                // sleep(1000);
                // robot.box.setPosition(0.02);
                // sleep(1000);
                
                robot.elevator.setTargetPosition(0);
                robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.elevator.setPower(0.7);                
                
            }
            // 1st level
            else if (gamepad1.b){
                curr_pos = robot.elevator.getCurrentPosition();
                
                telemetry.addData("curr_pos", curr_pos);
                telemetry.update();    
                
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
                sleep(1000);
                
                // robot.elevator.setTargetPosition(0);
                // robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // robot.elevator.setPower(0.7);
            }
            
            
            while (opModeIsActive() &&
                   robot.elevator.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path2",  "Running at %7d",
                                            robot.elevator.getCurrentPosition());
                telemetry.update();
            }
            
            robot.elevator.setPower(0.0);
            robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            curr_pos = robot.elevator.getCurrentPosition();
                
            telemetry.addData("curr_pos", curr_pos);

            // Send telemetry message to signify robot running;

            telemetry.addData("ARM_POWER", gamepad1.right_stick_x);
            //telemetry.addData("x", "%", x1);
            //telemetry.addData("yc", "%.2f", yc);
            //telemetry.addData("xc", "%.2f", xc);
            //telemetry.addData("y2", "%.2f", y2);            
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
