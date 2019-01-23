///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode.Autonomous;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//import org.firstinspires.ftc.teamcode.Library.MyOpModeNEW;
//
//import java.util.List;
//
//
//@Autonomous(name="TensorFlowCraterAndMarker", group="DogeCV")
//@Disabled
//public class TensorFlowCraterAndMarker extends MyOpModeNEW
//{
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//        hMap(hardwareMap);
//
//        initVuforia();
//
//        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//            initTfod();
//        } else {
//            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//        }
//
//        ElapsedTime time = new ElapsedTime();
//
////        while (!isStopRequested() && !imu.isGyroCalibrated()) {
////            sleep(50);
////            idle();
////            telemetry.addLine("Initializing IMU...");
////            telemetry.update();
////        }
//
//        markerDeploy.setPosition(0.65);
//        latch.setPosition(0.25);
//
//        rightBoxRotate.setPosition(.345);
//        leftBoxRotate.setPosition(.655);
//
//        waitForStart();
//
//
//        motorArmLeft.setPower(-0.5);
//        motorArmRight.setPower(0.5);
//        Thread.sleep(150);
//        motorArmLeft.setPower(0);
//        motorArmRight.setPower(0);
//
//
//        latch.setPosition(1.0);
//
//
//        Thread.sleep(750);
//
//        //try the dehang method with encoders that is in the library
//        motorArmLeft.setPower(0.5);
//        motorArmRight.setPower(-0.5);
//        Thread.sleep(850);
//        motorArmLeft.setPower(0);
//        motorArmRight.setPower(0);
//
//
//        Thread.sleep(500);
//
//        motorArmLeft.setPower(-0.5);
//        motorArmRight.setPower(0.5);
//        Thread.sleep(1000);
//        motorArmLeft.setPower(0);
//        motorArmRight.setPower(0);
//
//        setMotors(0.4,0.4);
//        Thread.sleep(750);
//        stopMotors();
//
//        gyroInit();
//
//        setMotors(-0.4,-0.4);
//        Thread.sleep(100);
//        stopMotors();
//
//        turnCorr(0.4, -12.5, 3000);
//        tfod.activate();
//
//        while(time.milliseconds() < 13500) {
//            sample();
//        }
//
//        turnCorr(0.4, 0, 3000);
//
//        //moveTo(-0.4,75,1000);
//        //center
//        if (position.equals("Center")) {
//            moveTo(-.3, 500, 4000);
//            turnCorr(.4, -25, 2000);
//            moveTo(0.3,350,2000);
//            turnCorr(0.4,-90,4000);
//            moveTo(0.3, 1000,4000);
//            turnCorr(0.4,-20,4000);
//            moveTo(0.3, 1000,4000);
//            markerDeploy.setPosition(0.2);
//            Thread.sleep(1000);
//            markerDeploy.setPosition(.65);
//            moveTo(-0.5,1500,5000);
//
//        } else if (position.equals("Left")) {
//            //left
//
//            turnCorr(0.4,45,4000);
//            moveTo(-0.4,500,3000);
//            turnCorr(0.4,-30,3000);
//            moveTo(-0.4,100, 2000);
//        } else if (position.equals("Right")) {
//            //right
//
//            turnCorr(0.4, -45,4000);
//            moveTo(-0.4,500);
//            turnCorr(0.4,30,2000);
//            moveTo(-0.4,75);
//            turnCorr(.4, -25, 2000);
//        }
//        tfod.shutdown();
//
////        telemetry.addData("Gold Position: ", position);
////        telemetry.update();
//    }
//}
