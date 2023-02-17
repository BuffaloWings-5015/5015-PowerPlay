package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.util.AxisDirection;
import org.firstinspires.ftc.teamcode.drive.util.BNO055IMUUtil;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
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

        double targetSlide;
        double targetV4b;
        double fbConst;
        fbConst = 0;
        targetSlide = 0;
        targetV4b = 0;

        waitForStart();
        float speedMultiplier = 1-gamepad1.right_trigger;
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x;//\ * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = 0; //-imu.getAngularOrientation().firstAngle;

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

            robot.leftFront.setPower(frontLeftPower*speedMultiplier);
            robot.leftBack.setPower(backLeftPower*speedMultiplier);
            robot.rightFront.setPower(frontRightPower*speedMultiplier);
            robot.rightBack.setPower(backRightPower*speedMultiplier);
            /*
            if (gamepad2.right_stick_y > 0) {
                if (robot.lSlide1.getCurrentPosition() < 10000) {
                    robot.lSlide1.setPower(gamepad2.right_stick_y);
                    robot.lSlide2.setPower(gamepad2.right_stick_y);
                    telemetry.addData("lslide encoder", robot.lSlide1.getCurrentPosition());
                    telemetry.addData("lslide encoder2", robot.lSlide2.getCurrentPosition());
                }
                if (gamepad2.a || gamepad2. b){
                    if (gamepad2.a){
                        robot.lSlide1.setPower(0.5);
                    }
                    if (gamepad2.b){
                        robot.lSlide1.setPower(-0.5);
                    }
                }
            } else if (gamepad2.right_stick_y < 0) {
                if (robot.lSlide1.getCurrentPosition() > 0) {
                    robot.lSlide1.setPower(gamepad2.right_stick_y);
                    telemetry.addData("lslide encoder", robot.lSlide1.getCurrentPosition());
                    telemetry.addData("lslide encoder2", robot.lSlide2.getCurrentPosition());
                }
                if (robot.lSlide2.getCurrentPosition() > 0) {
                    robot.lSlide2.setPower(gamepad2.right_stick_y);
                    telemetry.addData("lslide encoder", robot.lSlide1.getCurrentPosition());
                    telemetry.addData("lslide encoder2", robot.lSlide2.getCurrentPosition());
                }


            } else {
                robot.lSlide1.setPower(0);
                robot.lSlide2.setPower(0);
            }
             */
            //robot.v4bar1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                //robot.lSlide1.setPower(gamepad2.right_stick_y);


                if (gamepad2.left_bumper) {
                    robot.claw.setPower(0.5);
                }
                else if (gamepad2.right_bumper) {
                    robot.claw.setPower(-0.5);
                } else {
                    robot.claw.setPower(0);
                }

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

            if (gamepad2.dpad_left){
                targetSlide = 0;
                targetV4b = -500;
                fbConst = 0;
            }
            if (gamepad2.dpad_up){
                targetSlide = 450;
                targetV4b = -1250;
                fbConst = 0;
            }
            if (gamepad2.dpad_right){
                targetSlide = 800;
                targetV4b = -1250;
                fbConst = 0;
            }
            if (gamepad2.dpad_down){
                targetSlide = 0;
                targetV4b = 0;
                fbConst = -0.25;
            }
            final double KslideMulti;
            final double KV4bMulti;
            KslideMulti = 0.025;
            KV4bMulti = -0.005;
            if (true){
                robot.v4bar1.setPower((robot.v4bar1.getCurrentPosition() - targetV4b) * KV4bMulti + fbConst);
                robot.lSlide1.setPower((robot.lSlide1.getCurrentPosition() - targetSlide) * KslideMulti);
                robot.lSlide2.setPower((robot.lSlide2.getCurrentPosition() - targetSlide) * KslideMulti);
                telemetry.addData("linear slide encoder", robot.lSlide2.getCurrentPosition());
                telemetry.addData("V4Bar encoder", robot.v4bar1.getCurrentPosition());
                telemetry.addData("Dpad up", gamepad2.dpad_up);
                telemetry.addData("Dpad right", gamepad2.dpad_right);
                telemetry.addData("Dpad left", gamepad2.dpad_left);
                telemetry.addData("Dpad down", gamepad2.dpad_down);
                telemetry.update();

            }

        }
    }}