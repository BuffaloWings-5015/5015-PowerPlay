package org.firstinspires.ftc.teamcode;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        int v4barposition = 0;
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_X);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad2.right_stick_y > 0) {
                robot.lSlide1.setPower(gamepad2.right_stick_y);
                robot.lSlide2.setPower(gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y < 0) {
                robot.lSlide1.setPower(gamepad2.right_stick_y);
                robot.lSlide2.setPower(gamepad2.right_stick_y);
            } else {
                robot.lSlide1.setPower(0);
                robot.lSlide2.setPower(0);
            }
            {
                if (gamepad2.left_stick_y > 0) {
                    robot.v4bar1.setPower(gamepad2.left_stick_y);
                    robot.v4bar2.setPower(gamepad2.left_stick_y);
                } else if (gamepad2.left_stick_y < 0) {
                    robot.v4bar1.setPower(gamepad2.left_stick_y);
                    robot.v4bar2.setPower(gamepad2.left_stick_y);
                } else {
                    robot.v4bar1.setPower(0);
                    robot.v4bar2.setPower(0);
                }
                if (gamepad2.right_trigger > 0) {
                    robot.claw.setPower(0.5 * gamepad2.right_trigger);
                }
                if (gamepad2.left_trigger > 0) {
                    robot.claw.setPower(-0.5 * gamepad2.left_trigger);
                }
                double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                double x = gamepad1.right_stick_x; // Counteract imperfect strafing
                double rx = gamepad1.left_stick_x;
                double angle = 0;
                double newX,newY;
                newX = x * cos(angle) - y * sin(angle);
                newY = y * cos(angle) + x * sin(angle);
                robot.leftFront.setPower(newY + newX + rx);
                robot.rightFront.setPower(newY - newX - rx);
                robot.leftBack.setPower(newY - newX + rx);
                robot.rightBack.setPower(newY + newX - rx);

            }
        }
    }}