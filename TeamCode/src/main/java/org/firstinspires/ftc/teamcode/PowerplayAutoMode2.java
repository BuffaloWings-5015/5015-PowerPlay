package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.sun.tools.javac.comp.DeferredAttr;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Auto mode center pole SILLY EDITION 😂😂😂😂😂")
public class PowerplayAutoMode2 extends LinearOpMode { 

    String silly = "";
    FtcDashboard dashboard;
    //Definitions robot = new Definitions();
    PolePipeline detector;
    PipelineNew detector2;
    Definitions robot;


    enum autoStage {
        one,
        two
    }

    public double speedmulti = (double) 0.2;
    @Override
    public void runOpMode() throws InterruptedException {
        Definitions robot = new Definitions();
        robot.robotHardwareMapInit(hardwareMap);
        robot.driveInit();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(-90));
        Trajectory traj1 = drive.trajectoryBuilder(startPose, false)
                .back(48)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(),false)
                .strafeLeft(12)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end().plus(new Pose2d(0, 0, Math.toRadians(-90))),false)
                .forward(48)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(),false)
                .back(24)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(),false)
                .strafeRight(12)
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end(), false)
                .strafeLeft(12)
                .build();

        waitForStart();

        PipelineNew.ParkingPosition goopa = detector2.getPosition();
        sleep(1000);
        webcam.setPipeline(detector);
        dashboard = FtcDashboard.getInstance();

            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
            for (double i = 1; i < 5; i++){
            while (stage == autoStage.one){

            lineuppole:
            while (true){
            switch (detector.getCoords()) {
                case LEFT:
                    drive.setMotorPowers(-speedmulti, speedmulti, -speedmulti, speedmulti);
                    silly = "ma; 😡😡🤬🤬🤬🤬🤬  ";
                    break;
                case RIGHT:
                    drive.setMotorPowers(speedmulti, -speedmulti, speedmulti, -speedmulti);
                    // ... turn left
                    silly = "ma;   😡😡🤬🤬🤬🤬 ";
                    break;
                case CENTER:
                    // ...break
                    drive.setMotorPowers(0, 0, 0, 0);
                    silly = "b  ongo 👌👌👌👌👍👍👍👍🙃🔝G💯💯💯💯💯💯💯💯💯(messi(real????)";
                    break lineuppole;

            }
        }

            polewidther:
            while (true){
            switch (detector.getWidth()) {
                case CLOSE:
                    //  turn right

                    drive.setMotorPowers(-speedmulti, -speedmulti, -speedmulti, -speedmulti);
                    break;
                case FAR:
                    // ... turn left
                    drive.setMotorPowers(speedmulti, speedmulti, speedmulti, speedmulti);
                    break;
                case CENTER:
                    // ...break
                    drive.setMotorPowers(0, 0, 0, 0);
                    stage = autoStage.two;
                    break polewidther;

            }
            sertSlide(0.5,1000);
            sertBar(1.0, (int) (robot.ticks_in_degree * 180));
            sleep(1500);
            robot.claw.setPower(1);
            sleep(500);
            robot.claw.setPower(0);
            //TODO:slide up
            //TODO: find the encoders per inch for linear slide... NOW
            //bar up
            //drop and reset
            sertSlide(-0.5,000);
            sertBar(-1,000);
        } 
    } while (stage == autoStage.two){

        if (i==1){
            drive.turn(Math.toRadians(-90));
            drive.followTrajectory(traj3);
        } else if (i > 1){
            drive.followTrajectory(traj6);
        }



                    //grab
            drive.followTrajectory(traj4);
        }
                        sertSlide(0.5, (int) (1000 - (i * 20                                           )));
        drive.followTrajectory(traj3);
        robot.claw.setPower(1);

        //TODO:set slide and grab

        stage = autoStage.one;
    }
    }
    /*
    switch (goopa) {
        case NOT_FOUND:
            stop();

        case LEFT:
            drive.followTrajectory(traj5);
            break;

        case RIGHT:
            drive.followTrajectory(traj6);
            stop();
            break;

        case CENTER:
            stop();
    }



        webcam.stopStreaming();telemetry.addData("PolePOS", silly);telemetry.update();


        }

     */




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

}
