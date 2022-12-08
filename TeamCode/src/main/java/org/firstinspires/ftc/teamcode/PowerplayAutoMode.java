package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Sleeve Detector", group="Auto")
public class PowerplayAutoMode extends LinearOpMode {
    
    FtcDashboard dashboard;
    PipelineNew detector;
    private OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        OpenCvWebcam webcam;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //TODO: TUNE ROADRUNNER!!!! NOW
        Trajectory TrajectoryRight = drive.trajectoryBuilder(new Pose2d())
                .forward(10)
                .strafeRight(15)
                .build();
        Trajectory TrajectoryLeft = drive.trajectoryBuilder(new Pose2d())
                .forward(15)
                .strafeLeft(15)
                .build();
        Trajectory TrajectoryMiddle = drive.trajectoryBuilder(new Pose2d())
                .forward(15)
                .strafeLeft(15)
                .build();



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
                // do something
                drive.followTrajectory(TrajectoryMiddle);
                telemetry.addData("position",detector.getPosition());
                telemetry.update();
            case LEFT:
                // ...
                drive.followTrajectory(TrajectoryLeft);
                telemetry.addData("position",detector.getPosition());
                telemetry.update();
                break;
            case RIGHT:
                // ...
                drive.followTrajectory(TrajectoryRight);
                telemetry.addData("position",detector.getPosition());
                telemetry.update();
                break;
            case CENTER:
                drive.followTrajectory(TrajectoryMiddle);
                telemetry.addData("position",detector.getPosition());
                telemetry.update();
            default:
                telemetry.addData("position",detector.getPosition());
                telemetry.update();
                drive.followTrajectory(TrajectoryMiddle);
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

}}