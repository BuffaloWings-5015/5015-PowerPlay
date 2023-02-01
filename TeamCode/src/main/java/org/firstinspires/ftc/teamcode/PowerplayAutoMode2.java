package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Auto mode center pole")
public class PowerplayAutoMode2 extends LinearOpMode {

    FtcDashboard dashboard;
    Definitions robot = new Definitions();
    PolePipeline detector;
    private OpenCvCamera webcam;
    private DcMotor motor1 = null;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1  = hardwareMap.get(DcMotor.class, "motor");
        OpenCvWebcam webcam;
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam 1"), cameraMonitorViewId);
        PolePipeline detector = new PolePipeline(telemetry);
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


        sleep(1000);
        dashboard = FtcDashboard.getInstance();
        while (detector.getCoords() != PolePipeline.PolePosition.CENTER){
            switch (detector.getCoords()) {
                case LEFT:
                    //  turn right
                    motor1.setPower(.5);
                    break;
                case RIGHT:
                    // ... turn left
                    
                    break;
                case CENTER:
                    // ...break
                    motor1.setPower(0);
                    break;
            }
        }
        handleDashboard();
        }
    private void handleDashboard() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Frame Count", webcam.getFrameCount());
        packet.put("FPS", String.format("%.2f", webcam.getFps()));
        packet.put("Total frame time ms", webcam.getTotalFrameTimeMs());
        packet.put("Pipeline time ms", webcam.getPipelineTimeMs());
        packet.put("Overhead time ms", webcam.getOverheadTimeMs());
        packet.put("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

        dashboard.sendTelemetryPacket(packet);
    }

}