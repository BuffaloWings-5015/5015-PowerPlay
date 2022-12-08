package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class PIDF_Arm extends OpMode {
    Definitions robot = new Definitions();
    private PIDController controller;
    //TODO: tune in ftc dashboard
    public static double kP = 0, kI = 0, kD = -12.0;
    public static double kf = 0;

    public static int target = 0;





    @Override
    public void init() {
        controller = new PIDController(kP, kI, kD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



    }

    @Override
    public void loop() {
        controller.setPID(kP, kI, kD);
        int armPos = robot.v4bar1.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / robot.ticks_in_degree)) * kf;

        double power = pid + ff;

        robot.v4bar2.setPower(power);
        robot.v4bar1.setPower(power);

        telemetry.addData("pos,", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
