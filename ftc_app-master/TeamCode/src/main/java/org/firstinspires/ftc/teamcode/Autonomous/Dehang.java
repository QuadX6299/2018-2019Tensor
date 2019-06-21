package org.firstinspires.ftc.teamcode.Autonomous;///* Copyright (c) 2017 FIRST. All rights reserved.
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

//package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Library.MyOpModeNEW;
//

@Autonomous(name="Dehang", group="DogeCV")
@Disabled
public class Dehang extends MyOpModeNEW
{


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hMap(hardwareMap);
        //String position = "";


        ElapsedTime time = new ElapsedTime();




//        while (!isStopRequested() && !imu.isGyroCalibrated()) {
//            sleep(50);
//            idle();
//            telemetry.addLine("Initializing IMU...");
//            telemetry.update();
//        }

        markerDeploy.setPosition(0.65);
        //latch.setPosition(0.0);

        waitForStart();

        motorArmLeft.setPower(-0.5);
        motorArmRight.setPower(0.5);
        Thread.sleep(250);
        motorArmLeft.setPower(0);
        motorArmRight.setPower(0);

        while(time.milliseconds()<2000) {
            latch.setPosition(0.75);
        }

        Thread.sleep(1500);

        //try the dehang method with encoders that is in the library
        motorArmLeft.setPower(0.5);
        motorArmRight.setPower(-0.5);
        Thread.sleep(835);
        motorArmLeft.setPower(0);
        motorArmRight.setPower(0);


        Thread.sleep(1000);

        motorArmLeft.setPower(-0.5);
        motorArmRight.setPower(0.5);
        Thread.sleep(850);
        motorArmLeft.setPower(0);
        motorArmRight.setPower(0);

        moveTo(0.4,75);


        //turnCorr(0.4,-45,2000);

        telemetry.addData("latchPos: ", latch.getPosition());
        telemetry.update();
//        telemetry.addData("Gold Position: ", position);
//        telemetry.update();
    }
}
