package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.util.AxisDirection;
import org.firstinspires.ftc.teamcode.drive.util.BNO055IMUUtil;

@TeleOp
public class BasicOpMode_Iterative extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        Definitions robot = new Definitions();
        robot.robotHardwareMapInit(hardwareMap);
        robot.driveInit();
        double v4barpower = 0;
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests


        // Retrieve the IMU from the hardware map


        waitForStart();
        float speedMultiplier = 1-gamepad1.right_trigger;

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            while (opModeIsActive()) {
                v4barpower = gamepad2.left_stick_y;
                if (v4barpower > 0) {
                    v4barpower = Math.pow(v4barpower, 2);
                } else if (v4barpower < 0) {
                    v4barpower = v4barpower * -1;
                    v4barpower = Math.pow(v4barpower, 2);
                }
                double y =gamepad1.left_stick_y; // Remember, this is reversed!
                double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = -gamepad1.right_stick_x;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                robot.leftFront.setPower(frontLeftPower);
                robot.leftBack.setPower(backLeftPower);
                robot.rightFront.setPower(frontRightPower);
                robot.rightBack.setPower(backRightPower);
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
            robot.v4bar1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.v4bar2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.lSlide1.setPower(gamepad2.right_stick_y);


            if (gamepad2.right_bumper) {
                robot.claw.setPower(0.5);
            }
            else if (gamepad2.left_bumper) {
                robot.claw.setPower(-0.5);
            } else {
                robot.claw.setPower(0);
            }

            float v4barspeed;
            v4barspeed = 1 - gamepad2.right_trigger;
            {
                if (gamepad2.left_stick_y != 0) {
                    robot.v4bar1.setPower(v4barpower);
                    robot.v4bar2.setPower(v4barpower);
                    robot.v4bar3.setPower(v4barpower);
                }
                else {
                    robot.v4bar1.setPower(0);
                    robot.v4bar2.setPower(0);
                }
                telemetry.update();
            }
        }
    }}}