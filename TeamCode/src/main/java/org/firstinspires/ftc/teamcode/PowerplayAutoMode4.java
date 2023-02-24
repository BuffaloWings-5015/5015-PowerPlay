package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MotionLibrary.movement.MecanumDriveEncoders;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Test Center 2 😂😂😂😂😂")
public class PowerplayAutoMode4 extends LinearOpMode {

    String silly = "";
    FtcDashboard dashboard;

    PolePipeline detector;
    PipelineNew detector2;



    enum autoStage {
        one,
        two
    }

    double d_left, d_right, d_front;
    Definitions robot;

    public double speedmulti = (double) 0.3;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Definitions();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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




        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-36.79, -65.00, Math.toRadians(270.00)))
                .lineToLinearHeading(new Pose2d(-36.56, -21.99, Math.toRadians(270.00)))
                .lineTo(new Vector2d(-25.38, -21.25))
                .build();
        drive.setPoseEstimate(untitled0.start());
        waitForStart();
        robot.claw.setPower(1);
        sleep(1000);
        robot.lSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.v4bar1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.lSlide1.setTargetPosition(800);
        robot.lSlide2.setTargetPosition(800);
        robot.v4bar1.setTargetPosition(1200);
        robot.lSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.v4bar1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lSlide1.setPower(-1);
        robot.lSlide2.setPower(-1);
        robot.v4bar1.setPower(1);
        drive.followTrajectorySequence(untitled0);
        sleep(250);
        robot.lSlide1.setPower(0);
        robot.lSlide2.setPower(0);
        robot.v4bar1.setPower(0);
        robot.lSlide1.setTargetPosition(0);
        robot.lSlide2.setTargetPosition(0);
        robot.v4bar1.setTargetPosition(0);
        robot.lSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.v4bar1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lSlide1.setPower(0.5);
        robot.lSlide2.setPower(0.5);
        robot.v4bar1.setPower(1);
        sleep(10000);
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
        public double[] calculateRobotPose(double d_left, double d_front, double d_right) {
            double x, y, theta;

            theta = robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
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