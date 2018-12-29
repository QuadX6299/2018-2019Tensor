package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Library.MyOpMode;
import org.firstinspires.ftc.teamcode.Library.MyOpModeNEW;

@TeleOp(name="GATeleop", group="Linear Opmode")
public class GAT extends MyOpModeNEW {
    // Declare OpMode members.
    double gamepadLeftY; //Forwards/Backwards Movement
    double gamepadLeftX; //Strafing Movement
    double gamepadRightX; //Turning Movement
    double gamepadRightY;

    double AngleLStick; //Left Stick Movement Control
    double AngleRStick; //Right Stick Movement Control


    double turnSpeed = .5; //Speed Manipulated by buttons.

    boolean slow = false;


    double liner = 0;
    double straf = 0;
    double turno = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        hwMapTroll(hardwareMap);

        ElapsedTime delay = new ElapsedTime();
        delay.reset();

        align = false;

        resetStartTime();
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();

            if (liner != 0 || straf != 0 || turno != 0) {
                setMotorsAll(liner, straf, turno);
            }
            else {
                stopMotors();
            }

            if (!slow) {
                gamepadLeftY = gamepad1.left_stick_y;
                gamepadLeftX = gamepad1.left_stick_x;
                gamepadRightX = gamepad1.right_stick_x;
                gamepadRightY = gamepad1.right_stick_y;
                turnSpeed = 0.5;
            } else {
                gamepadLeftY = gamepad1.left_stick_y * .35;
                gamepadLeftX = gamepad1.left_stick_x * .35;
                gamepadRightX = gamepad1.right_stick_x * .35;
                turnSpeed = .15;
            }

            AngleLStick = Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x));
            if (AngleLStick < 0) {
                AngleLStick += 360;
            }
            AngleRStick = Math.toDegrees(Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x));
            //In order for the turning to adhere to the plane we start on, we need to reflect our x values.
            if (AngleRStick < 0) {
                AngleRStick += 360;
            }

            telemetry.addData("slow", slow);
            telemetry.addData("turn speed", turno);
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            double telemA = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
//            telemetry.addData("Gyro", telemA);
            telemetry.update();

            /**Movement (Gamepad 1: Left Stick, Right Stick, DPAD, b) */
            if (45 <= AngleLStick && AngleLStick <= 135 || 225 <= AngleLStick && AngleLStick <= 315) { //DRIVING FORWARDS/BACKWARDS
//                motorFL.setPower(gamepadLeftY);
//                motorBL.setPower(gamepadLeftY);
//                motorFR.setPower(-gamepadLeftY);
//                motorBR.setPower(-gamepadLeftY);
                liner = -1*gamepadLeftY;
                //setMotors(gamepadLeftY, gamepadLeftY);
            } else {
                liner = 0;
            }
            if (315 < AngleLStick && AngleLStick < 360 || 0 <= AngleLStick && AngleLStick < 45 || 135 < AngleLStick && AngleLStick < 225) { //STRAFING LEFT/RIGHT
                if (0 < gamepadLeftX && gamepadLeftX <= .1) { //Make sure the motors don't run at too low a speed.
//                    gamepadLeftX = .25;
                    straf = 1.0;
                    liner = 0;
                } else if (-.1 < gamepadLeftX && gamepadLeftX < 0) { //Make sure the motors don't run at too low a speed.
//                    gamepadLeftX = -.25;
                    straf = -1.0;
                    liner = 0;
                }
//                setMotorStrafe(gamepadLeftX);
                straf = gamepadLeftX;
            }
            else {
                straf = 0;
            }

            if (Double.isNaN(AngleRStick) || (AngleRStick == 180 && gamepad1.right_stick_x != 1)) {
                turno=0; }
            else if (liner != 0 || straf != 0) {
                turno = gamepadRightX;
            }
            else {
                turno = gamepadRightX; }

//            if (gamepad1.right_stick_x < -.05 || gamepad1.right_stick_x > .05 ) {
//                motorFL.setPower(-gamepad1.right_stick_x);
//                motorBL.setPower(-gamepad1.right_stick_x);
//                motorFR.setPower(-gamepad1.right_stick_x);
//                motorBR.setPower(-gamepad1.right_stick_x);
//            } else {
//                motorFL.setPower(0);
//                motorBL.setPower(0);
//                motorFR.setPower(0);
//                motorBR.setPower(0);
//            }


            //Increasing/decreasing strafing power

            /**End of Movement
             * Movement Modifiers (Gamepad 1: a,b,x,y)
             * Toggles to switch between slow mode for strafe and normal drive */





         }
        }
    }






