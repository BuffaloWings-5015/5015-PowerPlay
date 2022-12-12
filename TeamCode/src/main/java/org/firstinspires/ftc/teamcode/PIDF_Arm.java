package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class PIDF_Arm extends OpMode {
    Definitions robot = new Definitions();
    private PIDController controller;
    private DcMotorEx arm;
    private DcMotorEx arn2;
    //TODO: tune in ftc dashboard
    public static double kP = 0.005, kI = 0, kD = -0.5;
    public static double kf = 0;

    public static int target = 500;

    //PID Measurements
    double armPos;



    @Override
    public void init() {
        controller = new PIDController(kP, kI, kD);
        arm = hardwareMap.get(DcMotorEx.class, "v4bar1");
        arn2 = hardwareMap.get(DcMotorEx.class, "v4bar2");
        arn2.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



    }

    boolean armDown, armUp, armHold;
    double holdTarget, targetArm;

    @Override
    public void loop() {
        armPos = arm.getCurrentPosition() / 288.0 / 360;

        //Arm buttons
        if (gamepad1.b) {
            armUp = true;
            armDown = false;
        }
        if (gamepad1.x) {
            armUp = false;
            armDown = true;
        }

        if(armUp) {
            armPIDMove(180, 0.5, 0);
            if(armPos == 180) {
                armUp = false;
            }
        }

        if(armDown) {
            armPIDMove(0, 0.5, -0.05);
            if(armPos == 0) {
                armDown = false;
            }
        }

        /*
        if(!armUp && !armDown) {
            armHold = true;
            holdTarget = armPos;
        }
         */

        if(armHold) {
            armPIDMove(holdTarget, 0.5, 0);
        }

        telemetry.addData("pos,", armPos);
        telemetry.addData("target", targetArm);
        telemetry.addData("X", gamepad1.x);
        telemetry.addData("B", gamepad1.b);
        telemetry.update();
    }

    public void armPIDMove(double armTarget, double KP, double KD) {
        controller.setPID(KP, 0, KD);

        targetArm = armTarget;

        //armPos = arm.getCurrentPosition() / 288.0;
        double pid = controller.calculate(armPos, targetArm);
        //double ff = Math.cos(Math.toRadians(360 / target)) * kf;

        double power = pid;

        arm.setPower(power);
        arn2.setPower(power);
    }
}
