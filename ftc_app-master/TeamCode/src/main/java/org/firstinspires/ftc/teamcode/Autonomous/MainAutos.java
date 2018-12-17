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
//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.DogeCV;
//import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Library.MyOpModeNEW;
//
//
//@Autonomous(name="MainAutos", group="DogeCV")
//
//public class MainAutos extends MyOpModeNEW
//{
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//        hMap(hardwareMap);
//        String position = "";
//
//        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");
//
//        ElapsedTime time = new ElapsedTime();
//
//
//
//
//        while (!isStopRequested() && !imu.isGyroCalibrated()) {
//            sleep(50);
//            idle();
//            telemetry.addLine("Initializing IMU...");
//            telemetry.update();
//        }
//
//        markerDeploy.setPosition(0.65);
//
//        waitForStart();
//
//
//
//
//
//        moveTo(-0.4,75,1000);
//        if (detector.getAligned()) {
//
//            moveTo(-.3, 1350, 4000);
//            markerDeploy.setPosition(0.2);
//            Thread.sleep(1000);
//            markerDeploy.setPosition(.65);
//            turnCorr(.4, 45, 4000);
//            moveTo(.4,1800);
//            markerDeploy.setPosition(0.0);
//        } else if (!detector.getAligned() && detector.getXPosition() < 200){
//
//            turnCorr(0.4,45,4000);
//            moveTo(-0.4,500,3000);
//            turnCorr(0.4,-30,3000);
//            moveTo(-0.4,775);
//            markerDeploy.setPosition(0.0);
//            Thread.sleep(1000);
//            markerDeploy.setPosition(.65);
//            turnCorr(0.4, 45,3000);
//            moveTo(.4, 1800);
//        } else if (!detector.getAligned() && detector.getXPosition() > 400){
//
//            turnCorr(0.4, -45,4000);
//            moveTo(-0.4,500);
//            turnCorr(0.4,30,2000);
//            moveTo(-0.4,800);
//            markerDeploy.setPosition(0.0);
//            Thread.sleep(1000);
//            markerDeploy.setPosition(.65);
//            turnCorr(0.4, 50, 1500);
//            moveTo(.4,1900);
////            turnCorr(0.4, -45, 2500);
////            moveTo(-0.4, 200, 1000);
////            turnCorr(0.4,90,3500);
////            moveTo(-0.4, 750, 2000);
////            turnCorr(0.4,-135,4000);
////            moveTo(-0.5,1000,4000);
//        } else {
//            moveTo(-.3, 1350, 4000);
//            markerDeploy.setPosition(0.2);
//            Thread.sleep(1000);
//            markerDeploy.setPosition(.65);
//            turnCorr(.4, 45, 4000);
//            moveTo(.4,1800);
//            markerDeploy.setPosition(0.0);
//        }
//
//
//        detector.disable();
//
//        telemetry.addData("Gold Position: ", position);
//        telemetry.update();
//    }
//
//
//}
