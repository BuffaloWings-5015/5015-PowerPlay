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
    //TODO: tune in ftc dashboard
    public static double kP = 0.0, kI = 0, kD = 0;
    public static double f = 0;
    public static int target = 500;

    //PID Measurements
    double armPos;



    @Override
    public void init() {
        controller = new PIDController(kP, kI, kD);
        robot.robotHardwareMapInit(hardwareMap);
        robot.driveInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



    }

    @Override
    public void loop() {
        armPos = robot.v4bar1.getCurrentPosition();
        
        controller.setPID(KP, kI, KD);

        targetArm = armTarget;

        //armPos = arm.getCurrentPosition() / 288.0;
        double pid = controller.calculate(armPos, targetArm);
        double ff = Math.cos(Math.toRadians(target/robot.ticks_in_degree) * f);

        double power = pid + ff;

        robot.v4bar1.setPower(power);
        //Arm buttons
      
        telemetry.addData("pos,", armPos);
        telemetry.addData("target", targetArm);
        telemetry.update();
    }


        
    
}
