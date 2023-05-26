package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PIDF_Arm.target;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import ftc.rogue.blacksmith.BlackOp;
import ftc.rogue.blacksmith.Scheduler;
import ftc.rogue.blacksmith.annotations.CreateOnGo;
import ftc.rogue.blacksmith.annotations.EvalOnGo;
import ftc.rogue.blacksmith.listeners.Listener;
import ftc.rogue.blacksmith.listeners.ReforgedGamepad;

@TeleOp(name = "Groagas")
public class BlacksmithTest extends BlackOp {
    @CreateOnGo
    Definitions robot = new Definitions();

    @CreateOnGo(passHwMap = true)
    private SampleMecanumDrive drive;

    ReforgedGamepad driver = new ReforgedGamepad(gamepad1);
    ReforgedGamepad codriver = new ReforgedGamepad(gamepad2);

    @Override
    public void go() {
        Scheduler.launch(this, () -> {
            //pids loops
            driver.a.onRise(() -> {
                //funny lambda
                mTelemetry().addData("Gamepad1.a", "YES gamepad1.a");
            });
            driver.a.whileLow(() -> {
                //funny lambda
                mTelemetry().addData("Gamepad1.a", "NO gamepad1.a");
            });
            //gragas
            handleDrive();
        });
    }

    private void handleDrive() {
        Pose2d poseEstimate = drive.getPoseEstimate();

// Create a vector from the gamepad x/y inputs
// Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

// Pass in the rotated input + right stick value for rotation
// Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );
    }
    private void handlePID(){
        //TODO: add pid shenanigans
    }
}

