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
        double[] poseValues = calculateRobotPose(d_left, d_front, d_right);
        double x = poseValues[0];
        double y = poseValues[1];
        double heading = poseValues[2];

// Create a new Pose2d object with the calculated pose
Pose2d startingPose = new Pose2d(x, y, Math.toRadians(heading));
        //set to starting positon and angle.toCaps()

        /*
        robot.robotHardwareMapInit(hardwareMap);
        robot.driveInit();
        */


        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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
        TrajectorySequence Auto = drive.trajectorySequenceBuilder(new Pose2d(35.03, -62.03, Math.toRadians(90.00)))
        .lineTo(new Vector2d(33.15, -8.74), Math.toRadians(-45))
        .splineTo(new Vector2d(47.74, -12.64), Math.toRadians(-11.11))
        .splineTo(new Vector2d(59.15, -12.49), Math.toRadians(1.43))
        //fetch new cone
        .splineTo(new Vector2d(48.17, -12.64), Math.toRadians(0))
        //dunk new cone
        .splineTo(new Vector2d(33.44, -8.74), Math.toRadians(-45.00))
        .splineTo(new Vector2d(59.15, -12.49), Math.toRadians(1.43))
        .splineTo(new Vector2d(47.45, -13.36), Math.toRadians(-11.11))
        .splineTo(new Vector2d(59.15, -12.49), Math.toRadians(1.43))
        //fetch new cone
        .splineTo(new Vector2d(47.30, -13.07), Math.toRadians(0))
        //dunk new cone
        .splineTo(new Vector2d(32.71, -8.59), Math.toRadians(-45.00))
        .splineTo(new Vector2d(59.15, -12.49), Math.toRadians(1.43))
        .build();
        waitForStart();

        drive.followTrajectorySequence(auto);




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
        public static double[] calculateRobotPose(double d_left, double d_front, double d_right) {
            double x, y, theta;
        
            // Calculate x
            x = (60 - 8) - d_front * Math.sin(theta);
        
            // Calculate y
            y = (60 - 8) - d_left + d_right;
        
            // Calculate theta
            theta = Math.atan2(d_left - d_right, d_left + d_right);
        
            // Convert theta to degrees and adjust for coordinate system
            theta = Math.toDegrees(theta) - 90;
        
            // Return x, y, and theta as an array
            return new double[] { x, y, theta };
        }
        public void barMoveToEncoderPosition(double power, int targetEncoderPos) {
            // Reset the robot.lSlide1 encoder count to zero
            robot.lSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lSlide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        
            // Set the robot.lSlide1 target position to the specified encoder position
            robot.lSlide1.setTargetPosition(targetEncoderPos);
            robot.lSlide2.setTargetPosition(targetEncoderPos);
        
            // Set the robot.lSlide1 power to the specified power
            robot.lSlide1.setPower(power);
            robot.lSlide2.setPower(power);
        
            // Keep running the robot.lSlide1 until it reaches the target position
            while (robot.lSlide1.isBusy()) {
                // Wait for the robot.lSlide1 to reach the target position
                idle();
            }
        
            // Stop the robot.lSlide1 once it reaches the target position
            robot.lSlide1.setPower(0.0);
            robot.lSlide2.setPower(0.0);
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