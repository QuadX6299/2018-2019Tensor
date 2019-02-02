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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
@Autonomous(name = "GroundUpdate", group = "Concept")
public class GroundUpdate extends MyOpModeNEW {
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
        initVuforia();
        int goldMineralX = -1;
        String[] recentResults = new String[10];


        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        ElapsedTime time = new ElapsedTime();

        markerDeploy.setPosition(0.25);


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        setMotors(0.4, 0.4);
        Thread.sleep(750);
        stopMotors();

        gyroInit();

        setMotors(-0.4, -0.4);
        Thread.sleep(100);
        stopMotors();

        turnCorr(0.4, 14, 3000);


        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (time.milliseconds() < 10000) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    sleep(100);
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        int goldCount = 0;
                        int silverCount = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldCount++;
                                telemetry.addData("Gold " + goldCount + " Coordinates: ", (recognition.getLeft() + recognition.getRight()) / 2.0 + " " + (recognition.getTop() + recognition.getBottom()) / 2.0);
                            } else {
                                silverCount++;
                                telemetry.addData("Silver " + silverCount + " Coordinates: ", (recognition.getLeft() + recognition.getRight()) / 2.0 + " " + (recognition.getTop() + recognition.getBottom()) / 2.0);
                            }
                        }

                        ArrayList<Integer> removedInts = new ArrayList<>();
                        for (int i = 0; i < updatedRecognitions.size(); i++) {
                            Recognition rec = updatedRecognitions.get(i);
                            for (int j = 0; j < updatedRecognitions.size(); j++) {
                                Recognition rec2 = updatedRecognitions.get(j);
                                if (Math.abs((rec.getLeft() + rec.getRight()) / 2 - (rec2.getLeft() + rec.getRight()) / 2) < 150) {
                                    if (rec.getConfidence() < rec2.getConfidence()) {
                                        if (!removedInts.contains(i)) {
                                            removedInts.add(i);
                                        }
                                    }
                                    if (rec.getConfidence() > rec2.getConfidence()) {
                                        if (!removedInts.contains(j)) {
                                            removedInts.add(j);
                                        }
                                    }
                                }
                            }
                        }
                        Collections.sort(removedInts, Collections.reverseOrder());
                        for (int i : removedInts) {
                            updatedRecognitions.remove(i);
                        }

                        if (updatedRecognitions.size() == 3) {
                            goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    shiftArrayDown(recentResults, "L");
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    shiftArrayDown(recentResults, "R");
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    shiftArrayDown(recentResults, "C");
                                }
                            }
                        }
                        //telemetry.update();

                        if (updatedRecognitions.size() == 2) {
                            goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX == -1) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                target = "Right";
                                shiftArrayDown(recentResults, "R");
                            } else if (goldMineralX < silverMineral1X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                target = "Left";
                                shiftArrayDown(recentResults, "L");
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                target = "Center";
                                shiftArrayDown(recentResults, "C");
                            }
                        }
                        if (updatedRecognitions.size() == 1) {
                            goldMineralX = -1;
                            int silverMineral1X = -1;
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) (recognition.getLeft() + recognition.getRight() / 2);
                                }
                            }
                            if (goldMineralX < 500 && goldMineralX != -1) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                target = "Left";
                                shiftArrayDown(recentResults, "L");
                            }
                            if (goldMineralX > 500 && goldMineralX != -1) {
                                telemetry.addData("Gold Mineral Position", "Center");
                                target = "Center";
                                shiftArrayDown(recentResults, "C");
                            }
                            if (goldMineralX == -1) {
                                telemetry.addData("Gold Mineral Position", "Right (Sketch)");
                                target = "Right";
                                shiftArrayDown(recentResults, "R");
                            }
                        }
                    }

                }
                telemetry.addData("Recent results", Arrays.toString(recentResults));
                target = getTarget(recentResults);
                telemetry.addData("Gold X-Position", goldMineralX);
                telemetry.addData("Target", target);
                telemetry.update();
            }
            turnCorr(0.4, 0, 3000);

            //moveTo(-0.4,75,1000);
            //center
            if (target.equals("Center")) {
                moveTo(-.3, 850, 4000);
                markerDeploy.setPosition(0.8);
                Thread.sleep(1000);
                markerDeploy.setPosition(.2);
                turnCorr(.4, -50, 4000);
                moveTo(.4, 1800);
            } else if (target.equals("Left")) {
                //left
                turnCorr(0.4, 45, 4000);
                moveTo(-0.4, 500, 3000);
                turnCorr(0.4, -30, 3000);
                moveTo(-0.4, 150);
                markerDeploy.setPosition(0.8);
                Thread.sleep(1000);
                markerDeploy.setPosition(.2);
                turnCorr(0.4, -55, 3000);
                moveTo(.4, 1800);
            } else if (target.equals("Right")) {
                //right
                turnCorr(0.4, -45, 4000);
                moveTo(-0.4, 500);
                turnCorr(0.4, 30, 2000);
                moveTo(-0.4, 15);
                Thread.sleep(600);
                markerDeploy.setPosition(0.8);
                Thread.sleep(1000);
                markerDeploy.setPosition(.2);
                turnCorr(0.4, -55, 1500);
                moveTo(.4, 1900);
            }

        }
        tfod.shutdown();
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

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public String[] shiftArrayDown(String[] array, String insertEnd){
        for (int i = 0; i < array.length - 1; i++){
            array[i] = array[i + 1];
        }
        array[array.length - 1] = insertEnd;
        return array;
    }

    public String getTarget(String[] scores){
        int[] targetScores = {0, 0, 0};
        for (int i = 0; i < 10; i++){
            if (scores[i] != null) {
                if (scores[i].equals("L")) {
                    targetScores[0] += i;
                }
                if (scores[i].equals("C")) {
                    targetScores[1] += i;
                }
                if (scores[i].equals("R")) {
                    targetScores[2] += i;
                }
            }
        }
        if (targetScores[0] > targetScores[1] && targetScores[0] > targetScores[2]){
            return "Left";
        }
        else if (targetScores[1] > targetScores[2] && targetScores[1] > targetScores[0]){
            return "Center";
        }
        return "Right";
    }

}
