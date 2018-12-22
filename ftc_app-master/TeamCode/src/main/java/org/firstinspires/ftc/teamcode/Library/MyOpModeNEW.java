package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

public abstract class MyOpModeNEW extends LinearOpMode {

    public static BNO055IMU imu;
    public VuforiaLocalizer vuforia;
    public static final int MOVEMENT_DELAY = 300;


    public static DcMotor motorBL;
    public static DcMotor motorBR;
    public static DcMotor motorFL;
    public static DcMotor motorFR;

    public static DcMotor motorArmLeft;
    public static DcMotor motorArmRight;
    public static DcMotor motorBaseExtend; //base extend
    public static DcMotor manip;


    public static Servo leftBox;
    public static Servo rightBox;
    public static Servo leftBoxRotate;
    public static Servo rightBoxRotate;

    public static Servo mineralBlocker;

    public static Servo markerDeploy;
    public static Servo latch;

    public static ColorSensor leftBoxColor;
    public static ColorSensor rightBoxColor;

    public static Orientation angles;
    public static Acceleration gravity;

    public static ModernRoboticsI2cRangeSensor rangeR;
    public static ModernRoboticsI2cRangeSensor rangeL;
    public static ModernRoboticsI2cRangeSensor rangeF;


    private static final double COUNTS_PER_MOTOR_REV = 1440;  // AndyMark NeveRest 40:1 CPR
    private static final double DRIVE_GEAR_REDUCTION = 2.0;   // No gears, just motor shafts
    private static final double WHEEL_DIAMETER_INCHES = 3.0;   // 4" Omni wheels and 4" Stealth

    private static final double encoderInch = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159265);


    public char column;
    public boolean align;
    public double lastPow;
    public double turn;

    public double gyroError = 0;
    public ElapsedTime xDelay = new ElapsedTime();

    public String formatAngle(AngleUnit angleUnit, double angle) { //Formats the IMU Angle data into strings that can pass into telemetry.
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees) { //Formats the IMU Degree data into strings that can pass into telemetry.
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

//    public void composeTelemetry() { //Method that holds a majority of sensor telemetry for testing.
//        // At the beginning of each telemetry update, grab a bunch of data
//        // from the IMU that we will then display in separate lines.
//        telemetry.addAction(new Runnable() {
//            @Override
//            public void run() {
//                // Acquiring the angles is relatively expensive; we don't want
//                // to do that in each of the three items that need that info, as that's
//                // three times the necessary expense.
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////            gravity  = imu.getGravity();
//            }
//        });
//        telemetry.addLine()
//                .addData("Angle", new Func<String>() { //Position of the robots angle.
//                    @Override
//                    public String value() {
//                        return formatAngle(angles.angleUnit, angles.firstAngle); //Control Robot Pivot
//                    }
//                });
//        telemetry.addLine()
//                .addData("Left", new Func<String>() { //Left Range Sensor Telemetry.
//                    @Override
//                    public String value() {
//                        double localRange;
//                        double sensor = 0;
//                        localRange = rangeL.getDistance(DistanceUnit.INCH);
//                        if (!Double.isNaN(localRange) && (localRange < 1000)) {
//                            sensor = localRange;
//                        }
//                        return Double.toString(sensor);
//                    }
//                });
//        telemetry.addLine()
//                .addData("Right", new Func<String>() { //Right Range Sensor Telemetry.
//                    @Override
//                    public String value() {
//                        double localRange;
//                        double sensor = 0;
//                        localRange = rangeR.getDistance(DistanceUnit.INCH);
//                        if (!Double.isNaN(localRange) && (localRange < 1000)) {
//                            sensor = localRange;
//                        }
//                        return Double.toString(sensor);
//                    }
//                });
//
//        telemetry.addLine()
//                .addData("Front", new Func<String>() { //Front Range Sensor Telemetry.
//                    @Override
//                    public String value() {
//                        double localRange;
//                        double sensor = 0;
//                        localRange = rangeF.getDistance(DistanceUnit.INCH);
//                        if (!Double.isNaN(localRange) && (localRange < 1000)) {
//                            sensor = localRange;
//                        }
//                        return Double.toString(sensor);
//                    }
//                });
//    }


    public void hMap(HardwareMap type) { //Initialization of the Robot's hardware map in autonomous.
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");

        motorArmLeft = hardwareMap.dcMotor.get("motorArmLeft");
        motorArmRight = hardwareMap.dcMotor.get("motorArmRight");
        motorBaseExtend = hardwareMap.dcMotor.get("motorBaseExtend");

        leftBoxRotate = hardwareMap.servo.get("leftBoxRotate");
        rightBoxRotate = hardwareMap.servo.get("rightBoxRotate");

        //mineralBlocker = hardwareMap.servo.get("mineralBlocker");

//        leftBox = hardwareMap.servo.get("leftBox");
        rightBox = hardwareMap.servo.get("rightBox");


        markerDeploy = hardwareMap.servo.get("markerDeploy");
        latch = hardwareMap.servo.get("latch");

        //manip = hardwareMap.dcMotor.get("manip");

        motorArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;

        gyroInit();
    }

    public void hwMapTroll(HardwareMap type) {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");


        leftBox = hardwareMap.servo.get("leftBox");
        rightBox = hardwareMap.servo.get("rightBox");

        gyroInit();


    }
    public void hwMapManip(HardwareMap type) {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");

        leftBoxColor = hardwareMap.get(ColorSensor.class,"leftBoxColor");
        rightBoxColor = hardwareMap.get(ColorSensor.class,"rightBoxColor");

        leftBox = hardwareMap.servo.get("leftBox");
        rightBox = hardwareMap.servo.get("rightBox");

        gyroInit();
    }


    public void gyroInit() {
        BNO055IMU.Parameters Gparameters = new BNO055IMU.Parameters();
        Gparameters.mode = BNO055IMU.SensorMode.IMU;
        Gparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Gparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Gparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        Gparameters.loggingEnabled = true;
        Gparameters.loggingTag = "IMU";
        Gparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(Gparameters);
    }




    public void delay(long milliseconds) throws InterruptedException { //Delays the code via a sleep time.
        if (milliseconds < 0)
            milliseconds = 0;
        Thread.sleep(milliseconds);
    }

    public void setMotors(double left, double right) { //Moves forward when both values are positive.
        if (!opModeIsActive())
            return;

        motorFL.setPower(-left);
        motorBL.setPower(-left);
        motorFR.setPower(right);
        motorBR.setPower(right);
    }

    public void setMotorsAll(double linear, double strafe, double turn) {

        motorFL.setPower(-linear - turn - strafe);
        motorBL.setPower(-linear - turn + strafe);
        motorFR.setPower(linear - turn - strafe);
        motorBR.setPower(linear - turn + strafe);
    }

    public void stopMotors() { //Stops the motors.
        if (!opModeIsActive())
            return;

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }

    public void brakeMotors(){
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void brakeArm(){
        motorArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;
    }


    public void rangeMovePID(double inAway, ModernRoboticsI2cRangeSensor sensorVar) { //Moving forward/backwards using a Range Sensor.
        double sensor = sensorVar.getDistance(DistanceUnit.INCH);
        double pow;
        double localRange;
        while (((sensor < inAway - .35) || (sensor > inAway + .35)) && opModeIsActive() && align) { //While sensor isn't in the desired position, run.
            localRange = sensorVar.getDistance(DistanceUnit.INCH);
            if (gamepad1.x && xDelay.time() > .5) {
                stopMotors();
                align = false;
                xDelay.reset();
            }
            while ((Double.isNaN(localRange) || (localRange > 1000)) && opModeIsActive() && align) {
                if (gamepad1.x && xDelay.time() > .5) {
                    align = false;
                    xDelay.reset();
                } else if (!gamepad1.atRest()) { //Supposed to make the loop exit if joysticks are pressed.
                    stopMotors();
                    align = false;

                }

                //If a faulty value is detected, don't update our used variable till a good one is found.
                localRange = sensorVar.getDistance(DistanceUnit.INCH);
            }
            sensor = localRange; //Sets all working an  d usable values into a variable we can utilize.


            pow = .15;

            if (sensor > inAway) { //If the sensor value is greater than the target, move forward.
                setMotors(pow, pow);
            }
            if (sensor < inAway) { //If the sensor value is lower than than the target, move backwards.
                setMotors(-pow, -pow);
            }

            telemetry.addData("SensorValue", sensor); //Optional Telemetry
            telemetry.update();
        }
        stopMotors();
    }

    public void turnCorr(double pow, double deg, int timer) throws InterruptedException { //Turns to a desired angle using the IMU.
        if (!opModeIsActive()) //if the OpMode is not active, don't run.
            return;

        double newPow;
        double error;
        double errorMove;

        ElapsedTime time = new ElapsedTime();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)); // currPos is the current position

        delay(100);
        time.reset();

        while ((currPos > deg + 1 || currPos < deg - 1) && (time.milliseconds() < timer) && opModeIsActive()) { //While sensor isn't in the desired angle position, run.
            error = deg - currPos; //Finding how far away we are from the target position.
            errorMove = Math.abs(deg - currPos);
            if (error > 180) {
                error = error - 360;
            } else if (error < -180) {
                error = error + 360;
            }

            //Output = (Error * P) + (Error_Over_Time * I) + Bias
            newPow = (pow * (Math.abs(error) / 70)+.3); //Using the error to calculate our power.
            if (newPow < .15)
                newPow = .1;

            //The following code allows us to turn in the direction we need, and if we cross the axis
            //at which 180 degrees becomes -180, our robot can turn back in the direction which is closest
            //to the position we wish to be at (We won't make a full rotation to get to -175, if we hit 180).
            if (currPos < deg) {
                if (errorMove < 180) {
                    setMotors(-newPow, newPow); //Turns left
                }
                if (errorMove > 180) {
                    setMotors(newPow, -newPow); //Turns right if we go past the pos/neg mark.
                }
            } else if (currPos > deg) {
                if (errorMove < 180) {
                    setMotors(newPow, -newPow); //Turns right
                }
                if (errorMove > 180) {
                    setMotors(-newPow, newPow); //Turns left if we go past the pos/neg mark.
                }
            }
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("Gyro", currPos);
            telemetry.update();
        }
        stopMotors();
    }

    public void turnCorrIntegral(double pow, double deg, int timer) throws InterruptedException { //Turns to a desired angle using the IMU.
        if (!opModeIsActive()) //if the OpMode is not active, don't run.
            return;


        double newPow;
        double error = 1;
        double errorMove;
        double totalError = 0;
        double lastTime = 0;
        double kI = 0.1;

        ElapsedTime time = new ElapsedTime();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)); // currPos is the current position

        delay(100);
        time.reset();

        while ((currPos > deg + 1 || currPos < deg - 1) && (time.milliseconds() < timer) && opModeIsActive()) { //While sensor isn't in the desired angle position, run.

            error = deg - currPos;
            if (error > 180) {
                error = error - 360;
            } else if (error < -180) {
                error = error + 360;
            }
            totalError += (error * (time.milliseconds() - lastTime));
            lastTime = time.milliseconds();
            errorMove = Math.abs(deg - currPos);

            newPow = (pow * (Math.abs(error) / 70)) + (totalError * kI) + 0.3; //Using the error to calculate our power.
            if (newPow < .15)
                newPow = .1;



            if (currPos < deg) {
                if (errorMove < 180) {
                    setMotors(-newPow, newPow); //Turns left
                }
                if (errorMove > 180) {
                    setMotors(newPow, -newPow); //Turns right if we go past the pos/neg mark.
                }
            } else if (currPos > deg) {
                if (errorMove < 180) {
                    setMotors(newPow, -newPow); //Turns right
                }
                if (errorMove > 180) {
                    setMotors(-newPow, newPow); //Turns left if we go past the pos/neg mark.
                }
            }
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("Gyro", currPos);
            telemetry.update();
        }
        //Output = (Error * P) + (Error_Over_Time * I) + Bias
        //The following code allows us to turn in the direction we need, and if we cross the axis
        //at which 180 degrees becomes -180, our robot can turn back in the direction which is closest
        //to the position we wish to be at (We won't make a full rotation to get to -175, if we hit 180)


        stopMotors();

    }



    public double getGyroYaw(){
        double currPos;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        return currPos;
    }

    //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    public double setTurn(double deg) { //Turns to a desired angle using the IMU in teleop.
        double turnPow = 0;
        double error;
        double errorMove;
        double pd = .0055;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)); // currPos is the current position

        if ((currPos > deg + 1 || currPos < deg - 1) && opModeIsActive()) { //While sensor isn't in the desired angle position, run.
            error = deg - currPos; //Finding how far away we are from the target position.
            errorMove = Math.abs(deg - currPos);
            if (error > 180) {
                error = error - 360;
            } else if (error < -180) {
                error = error + 360;
            }

            //The following code allows us to turn in the direction we need, and if we cross the axis
            //at which 180 degrees becomes -180, our robot can turn back in the direction which is closest
            //to the position we wish to be at (We won't make a full rotation to get to -175, if we hit 180).
            if (currPos < deg) {
                if (errorMove < 180) {
                    turnPow = - Math.abs(pd * error); //Turns left
                }
                if (errorMove > 180) {
                    turnPow = Math.abs(pd * error); //Turns right if we go past the pos/neg mark.
                }
            } else if (currPos > deg) {
                if (errorMove < 180) {
                    turnPow = Math.abs(pd * error); //Turns right
                }
                if (errorMove > 180) {
                    turnPow = - Math.abs(pd * error); //Turns left if we go past the pos/neg mark.
                }
            }

        }
        if (turnPow < 0 && -.075< turnPow) {
            turnPow = -.075;
        }
        else if (turnPow > 0 && .075 > turnPow) {
            turnPow = .075;
        }
        return turnPow;
    }

//    public String WhiteOrYellow() {
//        if ((jewelColor.red() > 190) && (jewelColor.green()>190) && (jewelColor.blue() >190 ))
//        {
//            return "White";
//        }
//        return "Yellow";
//    }
//
//    public boolean isYellow(){
//        if((jewelColor.red() < 190) && (jewelColor.green()<190) && (jewelColor.blue() < 190 )){
//            return true;
//        }
//        return false;
//
//    }

//    public void mineralHit() {
//        //hits yellow mineral using color sensor
//        jewelArm.setPosition(.55);
//        jewelHand.setPosition(.4);
//        sleep(500);
//        jewelArm.setPosition(.1);
//        sleep(850);
//
//        jewelArm.setPosition(.55);
//        jewelHand.setPosition(.45);
//        sleep(300);
//        jewelHand.setPosition(.3);
//        sleep(300);
//    }

    public void deployMarker(double pos){
        //starting position:
        markerDeploy.setPosition(pos);
    }


    public int getEncoderAverageBase(){
        int encoderSum = Math.abs(motorBL.getCurrentPosition()) + Math.abs(motorBR.getCurrentPosition());

        return encoderSum / 2;
    }

    public int getEncoderAverageArm(){

        return motorArmLeft.getCurrentPosition();
    }


    public void  arcTurn(double pow, double deg, boolean stop, int tim) throws InterruptedException {

        if (!opModeIsActive())
            return;

        double newPow;

        ElapsedTime time = new ElapsedTime();

        resetGyro();
        delay(MOVEMENT_DELAY);
        time.reset();

        if (deg + gyroError > 0) {
            while (opModeIsActive() && deg > getGyroYaw() + gyroError && time.milliseconds() < tim) {
                newPow = Math.abs(pow) * (Math.abs(deg - getGyroYaw()) / 70);

                if (newPow < .15)
                    newPow = .15;

                if (pow > 0)
                    setMotors(newPow, 0);
                else
                    setMotors(0, -newPow);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        } else {
            while (opModeIsActive() && deg < getGyroYaw() + gyroError && time.milliseconds() < tim) {
                newPow = Math.abs(pow) * (Math.abs(deg - getGyroYaw()) / 70);


                if (newPow < .15)
                    newPow = .15;

                if (pow > 0)
                    setMotors(0, newPow);
                else
                    setMotors(-newPow, 0);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        }

        if (stop)
            stopMotors();
        delay(100);
        gyroError = getGyroYaw() + gyroError - deg;
    }


    //@TODO figure out how to make method adaptable for going both directions forwards and backwards

    public int getEncoderAvgExtend(){
        int encoderSum = Math.abs(motorBaseExtend.getCurrentPosition());
        return encoderSum;
    }



    public void resetGyro() {
        turn = imu.getAngularOrientation().firstAngle;
    }

    public void arcTurn(double pow, double deg) throws InterruptedException {
        arcTurn(pow, deg, true);
    }

    public void arcTurn(double pow, double deg, boolean stop) throws InterruptedException {
        arcTurn(pow, deg, stop, 6000);
    }




    public void setTurnAuto(double deg) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

        while (currPos > deg + 1 || currPos < deg - 1) { //while we are not at the right degree.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            setMotorsAll(0, 0, setTurn(deg));
            idle();
        }
    }

    public void extendRobot (int deg, double power, int tim) {
        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (opModeIsActive()) {

            if (deg > getEncoderAvgExtend() && time.milliseconds() < tim) {
                motorArmRight.setPower(power);
                motorArmLeft.setPower(power);
            } else if (Math.abs(deg) > getEncoderAvgExtend() && time.milliseconds() < tim) {
                motorArmRight.setPower(-power);
                motorArmLeft.setPower(-power);
            }
        }


    }


    public void setArm(double pow, int deg)
    {
        while (opModeIsActive() && deg != getEncoderAverageArm() + 10 || deg != getEncoderAverageArm() - 10)
        {
            if (deg < getEncoderAverageArm() - 10 )
            {
                motorArmLeft.setPower(-pow*(deg-getEncoderAverageArm()));
                motorArmRight.setPower(pow*(deg-getEncoderAverageArm()));
            } else if (deg > getEncoderAverageArm() + 10)
            {
                motorArmLeft.setPower(pow*(deg-getEncoderAverageArm()));
                motorArmRight.setPower(-pow*(deg-getEncoderAverageArm()));
            }
        }
    }


    public void moveTo(double pow, double deg) throws InterruptedException {
        moveTo(pow, deg, .6);
    }

    public void moveTo(double pow, double deg, double threshold) throws InterruptedException {
        moveTo(pow, deg, threshold, 0.9);
    }

    public void moveTo(double pow, double deg, double threshold, double red) throws InterruptedException {
        moveTo(pow, deg, threshold, red, 15000, true);
    }



    public void moveTo(double pow, double deg, double threshold, double red, int tim, boolean stop) throws InterruptedException {

        if (!opModeIsActive())
            return;

        ElapsedTime time = new ElapsedTime();


        resetGyro();
        resetEncoders();
        delay(MOVEMENT_DELAY);

        time.reset();

        if (deg > 0) {
            while (opModeIsActive() && deg > getEncoderAverageBase() && time.milliseconds() < tim) {
                if (getGyroYaw() + gyroError > threshold)
                    setMotors(pow / red, pow);
                else if (getGyroYaw() + gyroError < -threshold)
                    setMotors(pow, pow / red);
                else
                    setMotors(pow, pow);
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.addData("Encoder", getEncoderAverage());
//                telemetry.update();
//                Log.w("Gyro", "" + getGyroYaw());
                idle();
            }
        } else {
            while (opModeIsActive() && Math.abs(deg) > getEncoderAverageBase() && time.milliseconds() < tim) {
                if (getGyroYaw() + gyroError > threshold)
                    setMotors(-pow, -pow / red);
                else if (getGyroYaw() + gyroError < -threshold)
                    setMotors(-pow / red, -pow);
                else
                    setMotors(-pow, -pow);

                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Gyro Error", gyroError);
                telemetry.addData("Encoder", getEncoderAverageBase());
                telemetry.update();
                idle();
            }
        }
        if (stop)
            brakeMotors();

        gyroError = getGyroYaw() + gyroError;
    }

    public void resetEncoders() {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

