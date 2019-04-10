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

package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Library.MyOpModeNEW;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="RealTeleop", group="Linear Opmode")

public class RealTeleop extends MyOpModeNEW {

    double gamepad_left;
    double gamepad_right;
    double servoWait = 0;
    boolean isSlow = false;
    boolean turnSlow = false;
    boolean allSlow = false;
    double left = 1;
    double right = 1;

    int armStart = 0;
    int collectEncoder = 0;
    int scoreEncoder = 0;

    double deployPos = 0;


    @Override
    public void runOpMode() {

        hMap(hardwareMap);
        resetStartTime();

        //init servo
        markerDeploy.setPosition(.4);
        latch.setPosition(0.0);


        waitForStart();

        //armStart = motorArmLeft.getCurrentPosition();
        collectEncoder = armStart - 300;
        scoreEncoder = armStart-150;


        ElapsedTime servoDelay = new ElapsedTime();
        servoDelay.reset();

        // run until the end of the match (driver presses STOP)
        // Left stick forwards backwards; right stick turns left and right
        while (opModeIsActive()) {

            //collect
            if (gamepad1.b) {
                rightBoxRotate.setPosition(0.09);
                leftBoxRotate.setPosition(0.91);
            }
            //oscillate
            if (gamepad1.a) {
                rightBoxRotate.setPosition(.3);
                leftBoxRotate.setPosition(.7);
            }
            //store
            if (gamepad1.y) {
                rightBoxRotate.setPosition(0.0);
                leftBoxRotate.setPosition(1.0);
            }
            //deposit
            if (gamepad1.x) {
                if (servoDelay.milliseconds() > 100) {
                    rightBoxRotate.setPosition(rightBoxRotate.getPosition() + .045);
                    leftBoxRotate.setPosition(leftBoxRotate.getPosition() - .045);
                    servoDelay.reset();
                }
            }

            if (gamepad1.dpad_down)
            {
                mineralBlocker.setPosition(0.45);
            }

            if(gamepad1.dpad_up)
            {
                mineralBlocker.setPosition(0.9);
            }

//            if (gamepad1.b) {
//                rightBoxRotate.setPosition(.4);
//                leftBoxRotate.setPosition(.6);
//            }
            // flip robot front and back gamepad1.x @TODO flip code

            if (Math.abs(gamepad1.left_stick_y) > .05 ) {
                motorFL.setPower(gamepad1.left_stick_y);
                motorBL.setPower(gamepad1.left_stick_y);
                motorFR.setPower(-gamepad1.left_stick_y);
                motorBR.setPower(-gamepad1.left_stick_y);
            }

            if (gamepad1.right_stick_x < -.05 || gamepad1.right_stick_x > .05 ) {
                motorFL.setPower(-gamepad1.right_stick_x);
                motorBL.setPower(-gamepad1.right_stick_x);
                motorFR.setPower(-gamepad1.right_stick_x);
                motorBR.setPower(-gamepad1.right_stick_x);
            } else {
                motorFL.setPower(0.0);
                motorBL.setPower(0.0);
                motorFR.setPower(0.0);
                motorBR.setPower(0.0);
            }

            if (gamepad1.left_trigger > 0.05 ){
                motorBaseExtend.setPower(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.05){
                motorBaseExtend.setPower(-gamepad1.right_trigger);
            } else {
                motorBaseExtend.setPower(0.0);
            }

            if (gamepad1.left_bumper)
                manip.setPower(1.0);
            else if (gamepad1.right_bumper)
                manip.setPower(-1.0);
            else
                manip.setPower(0.0);

//            if (gamepad1.dpad_up) {
//                latch.setPosition(0.75);
//            } else if (gamepad1.dpad_down) {
//                latch.setPosition(0.0);
//            }

            //_____________________GAMEPAD 2 CODE_____________________________________________________


            if (Math.abs (gamepad2.left_stick_y) > 0.05){
                motorArmLeft.setPower(gamepad2.left_stick_y);
                motorArmRight.setPower(-gamepad2.left_stick_y);
            }else{
                motorArmLeft.setPower(0.0);
                motorArmRight.setPower(0.0);
            }

            if (gamepad2.a) {

                rightBoxRotate.setPosition(.5);
                leftBoxRotate.setPosition(.5);
            }

            if(gamepad2.right_bumper)
            {
                latch.setPosition(1.0);
            }

            if (gamepad2.x) {
                if (servoDelay.milliseconds() > 100) {
                    markerDeploy.setPosition(markerDeploy.getPosition() + .045);
                    servoDelay.reset();
                }
            }

            if (gamepad2.b) {
                if (servoDelay.milliseconds() > 100) {
                    markerDeploy.setPosition(markerDeploy.getPosition() - .045);
                    servoDelay.reset();
                }
            }


//            if (gamepad2.dpad_right)
//            {
//                setArm(0.75, armStart);
//            } else if (gamepad2.dpad_left) {
//                setArm(0.75,collectEncoder);
//            } else if (gamepad2.dpad_up) {
//                setArm(0.75, scoreEncoder);
//            } else {
//                motorArmLeft.setPower(0);
//                motorArmRight.setPower(0);
//            }

//            if (Math.abs(gamepad2.left_trigger) > .05 ) {
//                leftBoxRotate.setPower(-gamepad2.left_trigger);
//                rightBoxRotate.setPower(gamepad2.left_trigger);
//            }
//
//            if (Math.abs(gamepad2.right_trigger) > .05 ) {
//                leftBoxRotate.setPower(gamepad2.right_trigger);
//                rightBoxRotate.setPower(-gamepad2.right_trigger);
//            }




//
//            if(gamepad2.right_trigger > .05) {
//                rightBox.setPower(.5);
//                sleep(250);
//            } else if (gamepad2.right_bumper) {
//                rightBox.setPower(-.5);
//                sleep(250);
//            } else {
//                rightBox.setPower(0);
//            }
//
//
//            if(gamepad2.left_trigger > .05) {
//                leftBox.setPower(.5);
//                sleep(250);
//            } else if (gamepad2.left_bumper) {
//                leftBox.setPower(-.5);
//                sleep(250);
//            } else {
//                leftBox.setPower(0);
//            }

            telemetry.addData("allSlow: ", allSlow);
            telemetry.addData("turnSlow: ", turnSlow);
            telemetry.addData("ArmEncoder: ", getEncoderAverageArm());
//            telemetry.addData("leftBox: ", leftBox.getPosition());
//            telemetry.addData("rightBox: ", rightBox.getPosition());
//            telemetry.addData("leftBoxRotate: ", leftBoxRotate.getPosition());
//            telemetry.addData("rightBoxRotate: ", rightBoxRotate.getPosition());
            telemetry.update();
        }
    }
}
