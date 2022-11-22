package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@TeleOp
public class PIDF_Arm extends OpMode {
    Definitions robot = new Definitions();
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;





    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int armPos = robot.v4bar1.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / robot.ticks_in_degree)) * f;

        double power = pid + ff;

        robot.v4bar2.setPower(power);
        robot.v4bar1.setPower(power);

        telemetry.addData("pos,", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
