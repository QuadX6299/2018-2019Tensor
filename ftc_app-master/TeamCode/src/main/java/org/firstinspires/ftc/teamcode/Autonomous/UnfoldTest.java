/* Copyright (c) 2018 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Library.MyOpModeNEW;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "UnfoldTest", group = "Concept")
public class UnfoldTest extends MyOpModeNEW {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static String target = "Center";

    private static final String VUFORIA_KEY = "AfsLjeX/////AAABmUQCh0kvTE6ghhE9k6hRvhKDXeYFiILf2hzZdxqve5WufF/kXsVxFfdGWx4cv8N8R9XmndWbAIm3zTSNY6wS95DKDN89ZMaY9+ICrg9Yk5IhwKQJTYRL6hybkYAGiEsQVlgCoG9/CtDExYIo0ztEE4AITeq6OC9qejJcGZHNk3L+tke4VkKWHv2CSpamz77A2ul34WjTsuIjNrznEFS7UQLQCY/EKCTGuQnbrQn8P3xNSUauF4EzfX0npPRT1LE9KJEBsuYaZUH7erzUGxKS4uOD7G3DUSQv+V0WRaXiWYNWP5SvacaCuGsaA7rZeLp/AIYjPNY7eKUp37BOYYK89Vat6pt1fQ9D4A1g5YYEDK2m";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        hMap(hardwareMap);
        ElapsedTime time = new ElapsedTime();



        markerDeploy.setPosition(0.25);
        latch.setPosition(0.25);
        rightBoxRotate.setPosition(.345);
        leftBoxRotate.setPosition(.655);


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        rightBoxRotate.setPosition(0.15);
        leftBoxRotate.setPosition(0.85);

        motorArmLeft.setPower(0.5);
        motorArmRight.setPower(-0.5);
        Thread.sleep(1100);
        motorArmLeft.setPower(0);
        motorArmRight.setPower(0);


        Thread.sleep(100);


       // moveTo(0.4,20);


       // Thread.sleep(1000);


        rightBoxRotate.setPosition(.345);
        leftBoxRotate.setPosition(.655);

        Thread.sleep(100);

        motorArmLeft.setPower(0.5);
        motorArmRight.setPower(-0.5);
        Thread.sleep(1150);
        motorArmLeft.setPower(0);
        motorArmRight.setPower(0);

        Thread.sleep(250);

        setMotors(-0.4,-0.4);
        Thread.sleep(250);
        setMotors(0,0);

        Thread.sleep(250);

        motorArmLeft.setPower(0.5);
        motorArmRight.setPower(-0.5);
        Thread.sleep(475);
        motorArmLeft.setPower(0);
        motorArmRight.setPower(0);
        //up
        //servo init position
        Thread.sleep(250);

        motorArmLeft.setPower(-0.5);
        motorArmRight.setPower(0.5);
        Thread.sleep(200);
        motorArmLeft.setPower(0);
        motorArmRight.setPower(0);

        Thread.sleep(100);

        moveTo(0.4,200);

//        Thread.sleep(100);

//        rightBoxRotate.setPosition(0.15);
//        leftBoxRotate.setPosition(0.85);

        Thread.sleep(100);
        manip.setPower(1);
    }
}

