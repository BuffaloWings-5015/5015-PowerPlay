package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RegionalTeleop.PoleCenteringState.notMoving;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MotionLibrary.movement.MecanumDriveEncoders;
import org.firstinspires.ftc.teamcode.MotionLibrary.util.Vector2D;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name="Test Center 3 😂😂😂😂😂")
public class PowerplayAutoMode5 extends LinearOpMode {

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
    MecanumDriveEncoders motion;

    public double speedmulti = (double) 0.3;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status:", "runOpmode");
        telemetry.update();

        robot = new Definitions();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.robotHardwareMapInit(hardwareMap);
        motion = new MecanumDriveEncoders(this, 0);
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


        telemetry.addData("Status:", "cameraStuffs");
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //PolePipeline detector = new PolePipeline(telemetry);
        //webcam.setPipeline(detector);

        /*webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

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
          /*  }
        });
*/


        telemetry.addData("Status:", "trajectory Stuff");
        telemetry.update();

        telemetry.addData("Status :", "waiting for start");
        waitForStart();

        telemetry.addData("Status:", "following stuff");
        telemetry.update();

        motion.lineTo(new Vector2D(0, 60));



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


//        webcam.stopStreaming();
        telemetry.addData("PolePOS", silly);
        telemetry.update();


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
    Vector2D moveVector = new Vector2D();
    double poleCoordinate = 0;
    double poleDistance = 0;
    public void centerAlongPole() {
        while (poleCoordinate < 450 && poleCoordinate > 520) {
            break;
        }
        poleDistance = robot.distanceSensor.getDistance(DistanceUnit.CM);

        while (poleDistance > 9 && poleDistance < 11) {
            poleDistance = robot.distanceSensor.getDistance(DistanceUnit.CM);

            break;
        }
    }

}