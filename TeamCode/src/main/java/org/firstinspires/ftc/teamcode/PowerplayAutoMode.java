package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PipelineNew.ParkingPosition.CENTER;
import static org.firstinspires.ftc.teamcode.PipelineNew.ParkingPosition.LEFT;
import static org.firstinspires.ftc.teamcode.PipelineNew.ParkingPosition.RIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.nio.channels.Pipe;
import java.util.Arrays;

@Autonomous(name="Sleeve Detector", group="Auto")
public class PowerplayAutoMode extends LinearOpMode {

    FtcDashboard dashboard;
    Definitions robot = new Definitions();
    PipelineNew detector;
    private OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        OpenCvWebcam webcam;
        robot.robotHardwareMapInit(hardwareMap);
        robot.driveInit();
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam 1"), cameraMonitorViewId);
        PipelineNew detector = new PipelineNew(telemetry);
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();


        dashboard = FtcDashboard.getInstance();
        switch (detector.getPosition()) {
            case NOT_FOUND:
                goLeft();
                telemetry.addData("position",detector.getPosition());
                telemetry.update();
            case LEFT:
                // ...
                goLeft();
                telemetry.addData("position",detector.getPosition());
                telemetry.update();
                break;
            case RIGHT:
                // ...
                goRight();
                telemetry.addData("position",detector.getPosition());
                telemetry.update();
                break;
            case CENTER:
                goForward();
                telemetry.addData("position",detector.getPosition());
                telemetry.update();
            default:
                goForward();
                telemetry.addData("position",detector.getPosition());
                telemetry.update();
        }
        webcam.stopStreaming();
    }
    private void handleDashboard() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Frame Count", webcam.getFrameCount());
        packet.put("FPS", String.format("%.2f", webcam.getFps()));
        packet.put("Total frame time ms", webcam.getTotalFrameTimeMs());
        packet.put("Pipeline time ms", webcam.getPipelineTimeMs());
        packet.put("Overhead time ms", webcam.getOverheadTimeMs());
        packet.put("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        packet.put("Detected Position", detector.getPosition());

        dashboard.sendTelemetryPacket(packet);
    }
    private void goLeft() {
        robot.leftFront.setPower(-0.5);
        robot.rightFront.setPower(-0.5);
        robot.leftBack.setPower(-0.5);
        robot.rightBack.setPower(-0.5);
        sleep(1000);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        sleep(1000);
        robot.leftFront.setPower(0.5);
        robot.rightFront.setPower(-0.5);
        robot.leftBack.setPower(-0.5);
        robot.rightBack.setPower(0.5);
        sleep(1700);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        /*
        robot.leftFront.setPower(-0.5);
        robot.rightFront.setPower(-0.5);
        robot.leftBack.setPower(-0.5);
        robot.rightBack.setPower(-0.5);
        sleep(500);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
         */
    }
    private void goRight(){
        robot.leftFront.setPower(-0.5);
        robot.rightFront.setPower(-0.5);
        robot.leftBack.setPower(-0.5);
        robot.rightBack.setPower(-0.5);
        sleep(1100);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        sleep(1000);
        robot.leftFront.setPower(-0.5);
        robot.rightFront.setPower(0.5);
        robot.leftBack.setPower(0.5);
        robot.rightBack.setPower(-0.5);
        sleep(1300);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
    }
    private void goForward(){
        robot.leftFront.setPower(-0.5);
        robot.rightFront.setPower(-0.5);
        robot.leftBack.setPower(-0.5);
        robot.rightBack.setPower(-0.5);
        sleep(800);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
    }
}