package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Test Center 😂😂😂😂😂")
public class PowerplayAutoMode3 extends LinearOpMode {

    String silly = "";
    FtcDashboard dashboard;

    PolePipeline detector;
    PipelineNew detector2;


    enum autoStage {
        one,
        two
    }

    public double speedmulti = (double) 0.3;
    @Override
    public void runOpMode() throws InterruptedException {
        Definitions robot = new Definitions();
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.robotHardwareMapInit(hardwareMap);
        robot.driveInit();
        OpenCvCamera webcam;
        autoStage stage = autoStage.one;
        //set to starting positon and angle.toCaps()

        /*
        robot.robotHardwareMapInit(hardwareMap);
        robot.driveInit();
        */


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
        /*
        TrajectorySequence untitled = drive.trajectorySequenceBuilder(new Pose2d(-36.79, -65.23, Math.toRadians(268.21)))
                .lineToSplineHeading(new Pose2d(-35.66, -11.29, Math.toRadians(264.81)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-23.70, -11.29, Math.toRadians(268.53)))
                .build();
        drive.setPoseEstimate(untitled.start());
        TrajectorySequence untitled2 = drive.trajectorySequenceBuilder(new Pose2d(-23.70,-11.29,Math.toRadians(268.53)))
                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(-60.36, -11.96, Math.toRadians(180.00)))
                .build();
*/
        waitForStart();

        //drive.followTrajectorySequence(untitled);

        lineuppole:
        while (true){
            switch (detector.getCoords()) {
                case LEFT:
                    robot.leftFront.setPower(-0.3);
                    robot.leftBack.setPower(0.3);
                    robot.rightFront.setPower(0.3);
                    robot.rightBack.setPower(-0.3);
                    break;
                case RIGHT:
                    robot.leftFront.setPower(0.3);
                    robot.leftBack.setPower(-0.3);
                    robot.rightFront.setPower(-0.3);
                    robot.rightBack.setPower(0.3);
                    break;
                case CENTER:
                    // ...break
                    robot.leftFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.rightBack.setPower(0);
                    silly = "b  ongo 👌👌👌👌👍👍👍👍🙃🔝G💯💯💯💯💯💯💯💯💯(messi(real????)";
                    break lineuppole;

            }
        }

        polewidther:
        while (true) {
            switch (detector.getWidth()) {
                case FAR:
                    //  turn right
                    robot.leftFront.setPower(0.3);
                    robot.leftBack.setPower(0.3);
                    robot.rightFront.setPower(0.3);
                    robot.rightBack.setPower(0.3);

                    break;
                case CLOSE:
                    // ... turn left
                    robot.leftFront.setPower(-0.3);
                    robot.leftBack.setPower(-0.3);
                    robot.rightFront.setPower(-0.3);
                    robot.rightBack.setPower(-0.3);
                    break;
                case CENTER:
                    // ...break
                    robot.leftFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.rightBack.setPower(0);
                    break polewidther;

            }
        }



        dashboard = FtcDashboard.getInstance();
       /* horizpole:
        while (true){
            switch (detector.getCoords()) {
                case CENTER:
                    //
                    drive.setMotorPowers(0, 0, 0, 0);
                    break horizpole;
                case RIGHT:
                    //
                    drive.setMotorPowers(speedmulti, speedmulti, speedmulti, speedmulti);

                    break;
                case LEFT:
                    //
                    drive.setMotorPowers(-speedmulti, -speedmulti, -speedmulti, -speedmulti);
                    break;


            }
        }
/*
    switch (goopa) {
        case NOT_FOUND:
            ///////////
            stop();
        case LEFT:
            // ...
            drive.followTrajectory(traj5);
            break;
        case RIGHT:
            // ...
            drive.followTrajectory(traj6);
            stop();
            break;
        case CENTER:
            //////////
            stop();
    }
*/


        webcam.stopStreaming();telemetry.addData("PolePOS", silly);telemetry.update();


        }


/*
    private void sertSlide(double slidePower, int slideTop){
        robot.lSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);  
            robot.lSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);  
            robot.lSlide1.setTargetPosition(slideTop);
            robot.lSlide2.setTargetPosition(slideTop);
            robot.lSlide1.setPower(slidePower);
            robot.lSlide2.setPower(slidePower);
    }
    private void sertBar(double barPower, int barTop){
        robot.v4bar1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.v4bar1.setTargetPosition(barTop);
            robot.v4bar1.setPower(barPower);


    }
    */

}