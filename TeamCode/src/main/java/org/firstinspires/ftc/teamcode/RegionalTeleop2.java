package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PIDF_Arm.target;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "FieldCentricTeleop 😎😎😎😎")
public class RegionalTeleop2 extends LinearOpMode {
    Definitions robot;
    int burger = 0;
    int v4barburger = 0;

    @Override


    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        Definitions robot = new Definitions();
        robot.robotHardwareMapInit(hardwareMap);
        robot.driveInit();

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests


        // Retrieve the IMU from the hardware map
        BNO055IMU imu;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        //Gripper Controls
        double servoPower = 0;
        double timeSinceServoStateChange = 0;
        servoState lastServoState = servoState.closed;
        servoState currentServoState = servoState.closed;
        boolean servoAuto = false;
        boolean colorSensorLast = false;
        boolean colorSensorCurrent = false;
        double colorSensorCooldown = 0;

        //Slide Controls
        double targetSlide;
        double targetV4b;
        double fbConst;
        double targetV4bdegree = 0;

        fbConst = 0;
        targetSlide = 0;
        targetV4b = 0;
        robot.lSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.v4bar1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lSlide1.setTargetPosition(0);
        robot.lSlide2.setTargetPosition(0);
        robot.v4bar1.setTargetPosition(0);
        waitForStart();
        float speedMultiplier = 1 - gamepad1.right_trigger;
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //Movement
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x;//\ * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            robot.leftFront.setPower(frontLeftPower * speedMultiplier);
            robot.leftBack.setPower(backLeftPower * speedMultiplier);
            robot.rightFront.setPower(frontRightPower * speedMultiplier);
            robot.rightBack.setPower(backRightPower * speedMultiplier);


            //Servo Controller
            if (gamepad1.dpad_up) {
                currentServoState = servoState.opening;
                servoAuto = true;
            } else if (gamepad1.dpad_down) {
                currentServoState = servoState.closed;
                servoAuto = true;
            } else if (gamepad1.dpad_left) {
                currentServoState = servoState.none;
                servoAuto = false;
            }

            if (gamepad2.right_bumper || gamepad2.left_bumper) {
                currentServoState = servoState.none;
                servoAuto = false;
            }

            colorSensorCurrent = robot.colorsensor.alpha() <= 1;

            if (servoAuto) {
                if (colorSensorCurrent && colorSensorLast) {
                    colorSensorCooldown = System.currentTimeMillis();
                }
                if (colorSensorCurrent && System.currentTimeMillis() - colorSensorCooldown >= 250) {
                    currentServoState = servoState.closed;
                }
            }


            //4 bar and slides control
                /*
            float v4barspeed;
            v4barspeed = 1 - gamepad2.right_trigger;
            {
                if (gamepad2.left_stick_y != 0) {
                    robot.v4bar1.setPower(gamepad2.left_stick_y);
                }
                else {
                    robot.v4bar1.setPower(0);
                }
                telemetry.update();
            }*/

            //just p no i or d. (i got i gpu on streets last night btw)

            if (gamepad2.dpad_left) {
                burger = 0;
                v4barburger = 1200;
            }
            if (gamepad2.dpad_up) {
                burger = 1400;
                v4barburger = 1200;
            }
            if (gamepad2.dpad_right) {
                burger = 400;
                v4barburger = 1200;
            }
            if (gamepad2.dpad_down) {

                burger = 0;
                v4barburger = 0;
            }
            targetSlide += gamepad2.left_stick_y;
            targetV4b += gamepad2.right_stick_y * -8;


            final double KslideMulti;
            final double KV4bMulti;
            KslideMulti = -0.025;
            KV4bMulti = -0.001;

         /*
            if (targetSlide <= 0) {
                targetSlide = 0;
            }

          */
            /*
            if (targetV4b >= 0) {
                targetV4b = 0;
            }
             */

            if (gamepad2.a) {
                burger += 200;
            } else if (gamepad2.b) {
                burger -= 200;
            }

            //v4barburger 👀👀👀
            v4barburger += 100 * -gamepad2.left_stick_y;


            //Servo Controller
            switch (currentServoState) {
                case closed:
                    servoPower = 0.5;
                    break;
                case opening:
                    if (lastServoState != servoState.opening) {
                        timeSinceServoStateChange = System.currentTimeMillis();
                    } else {
                        if (System.currentTimeMillis() - timeSinceServoStateChange <= 125) {
                            servoPower = -0.5;
                        } else {
                            currentServoState = servoState.none;
                        }
                    }
                    break;
                case none:
                    if (gamepad2.right_bumper) {
                        servoPower = 0.5;
                    } else if (gamepad2.left_bumper) {
                        servoPower = -0.5;
                    } else {
                        servoPower = 0;
                    }
                    break;
            }

            robot.claw.setPower(servoPower);
            robot.lSlide1.setTargetPosition(burger);
            robot.lSlide2.setTargetPosition(burger);
            robot.v4bar1.setTargetPosition(v4barburger);
            robot.lSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.v4bar1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lSlide1.setPower(1);
            robot.lSlide2.setPower(1);
            robot.v4bar1.setPower(0.6 + (Math.cos(Math.toRadians(target / robot.ticks_in_degree) * 0.1)));

            telemetry.addData("linear slide encoder", robot.lSlide2.getCurrentPosition());
            telemetry.addData("V4Bar encoder", robot.v4bar1.getCurrentPosition());
            telemetry.addData("linear slide encoder target", targetSlide);
            telemetry.addData("V4Bar encoder target", targetV4b);
            telemetry.addData("Dpad up", gamepad2.dpad_up);
            telemetry.addData("Dpad right", gamepad2.dpad_right);
            telemetry.addData("Dpad left", gamepad2.dpad_left);
            telemetry.addData("Dpad down", gamepad2.dpad_down);
            telemetry.addData("Color Sensor alpha", robot.colorsensor.alpha());
            telemetry.addData("Servo State", currentServoState);
            telemetry.update();

            //used for servo control
            lastServoState = currentServoState;
            colorSensorLast = colorSensorCurrent;
        }


    }
//ttgttt

    public enum servoState {
        opening,
        closed,
        none;
    }
}