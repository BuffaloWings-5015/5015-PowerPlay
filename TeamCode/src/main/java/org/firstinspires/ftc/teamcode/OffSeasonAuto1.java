package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MotionLibrary.util.Ultrasonic;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="321321MGUXDZHFHVDNLKDJYF;LDMFNHC]PODWDFODSUJNHFOUGEBVFNH]RGONBV L")
public class OffSeasonAuto1 extends LinearOpMode {
    SampleMecanumDrive drive;
    Ultrasonic ultra;
    Ultrasonic ultra2;
    Ultrasonic ultra3;
    AnalogInput uSensor;
    AnalogInput uSensor2;
    double distance;
    double distance2;
    double distance3;
    public enum autoStates {
        ONE,
        TWO,
        THREE,
        GRAB1,
        LIFT1,
        IDLE,
        WAIT2,
        TWOPOINTFIVE,
        BACKWARD1,
        WAIT
    };
    BNO055IMU imu;
    @Override


    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        arm.driveInit();
        lift.driveInit();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        uSensor = hardwareMap.analogInput.get("input0");
        uSensor2 = hardwareMap.analogInput.get("input2");
        AnalogInput uSensor3 = hardwareMap.analogInput.get("input4");
        drive = new SampleMecanumDrive(hardwareMap);
        ultra = new Ultrasonic(uSensor);
        ultra2 = new Ultrasonic(uSensor2);
        ultra3 = new Ultrasonic(uSensor3);


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests


        // Retrieve the IMU from the hardware map
/*
        BNO055IMU imu;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception

      */  //imu.initialize(parameters);
        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())

                .strafeRight(16.5)

                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .forward(10)

                .build();
        Trajectory backward = drive.trajectoryBuilder(trajectory2.end())
                .back(10)
                .build();
        double waitTime1 = 1.5;

        ElapsedTime waitTimer1 = new ElapsedTime();
        ElapsedTime waitTimer2 = new ElapsedTime();
        ElapsedTime waitTimer3 = new ElapsedTime();

        double targetSlide;
        waitForStart();
        if (isStopRequested()) return;
        autoStates autoState = autoStates.ONE;
        drive.followTrajectoryAsync(trajectory1);
        lift.setTargetPositions(200);
        arm.setTargetPositions(0);


        /*
        robot.lSlide1.setTargetPosition(0);
        robot.lSlide2.setTargetPosition(0);
        robot.lSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lSlide1.setPower(-1);
        robot.lSlide2.setPower(-1);

         */
        while (opModeIsActive() && !isStopRequested()) {
            distance = ultra.getVoltage();
            double distance1 = uSensor.getVoltage();
            double distance2 = uSensor2.getVoltage();
            double qwr = ultra.getDistance();
            double qwre = ultra2.getDistance();
            double qwert = ultra3.getDistance();
            double[] posevalues = calculateRobotPose(qwr,qwre,qwert);


            switch(autoState) {
                case ONE:
                    if (!drive.isBusy()) {

                        autoState = autoStates.WAIT;
                        waitTimer1.reset();

                    }

                    break;
                case WAIT:
                    if (waitTimer1.seconds() >= waitTime1) {
                        autoState = autoStates.TWO;
                        drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case WAIT2:
                    if (waitTimer2.seconds() >= 1) {
                        autoState = autoStates.BACKWARD1;

                    }
                    break;
                case TWO:
                    if (!drive.isBusy()) {
                        autoState = autoStates.TWOPOINTFIVE;
                        claw.claw.setPower(1);
                        waitTimer2.reset();
                    }
                    break;
                case TWOPOINTFIVE:
                    if (waitTimer2.seconds() >= 1.5) {
                        autoState = autoStates.GRAB1;
                        lift.setTargetPositions(1200);
                        arm.setTargetPositions(200);
                        waitTimer2.reset();
                    }
                    break;
                case GRAB1:
                    if (waitTimer2.seconds() >= waitTime1) {
                        autoState = autoStates.BACKWARD1;
                        drive.followTrajectoryAsync(backward);
                    }
                    break;
                case LIFT1:
                    if (waitTimer3.seconds() >= 1) {
                        autoState = autoStates.BACKWARD1;
                        drive.followTrajectoryAsync(backward);
                    }
                    break;
                case BACKWARD1:
                    if (!drive.isBusy()){
                        autoState = autoStates.THREE;
                        arm.setTargetPositions(100);
                        lift.setTargetPositions(0);
                        claw.claw.setPower(-1);
                        waitTimer3.reset();
                    }
                    break;
                case THREE:
                    if (waitTimer3.seconds() >= 1) {
                        autoState = autoStates.IDLE;
                        claw.claw.setPower(0);
                    }
                    break;
                case IDLE:
                    break;
            }
            drive.update();
            lift.update();
            arm.update();




            Pose2d poseEstimate = drive.getPoseEstimate();
            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("stage: ", autoState);
            telemetry.addData("distance1", distance1);
            telemetry.addData("distance2", distance2);
            telemetry.addData("inches", qwr);
            telemetry.addData("inches2", qwre);
            telemetry.addData("inches3",qwert);
           // telemetry.addData("distance0", distance);
           telemetry.addData("posevalues",posevalues[0]);
            telemetry.addData("posevalues",posevalues[1]);
            telemetry.addData("posevalues",posevalues[2]);
            telemetry.update();
        }

    }
    public double[] calculateRobotPose(double d_left, double d_front, double d_right) {
        double x, y, theta;

        theta = -imu.getAngularOrientation().firstAngle;
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
    class Arm {
        public DcMotor arm;
        public Arm(HardwareMap Map) {
            arm = Map.get(DcMotorEx.class, "v4bar");
        }
        public void driveInit(){
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        public void update(){
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
        }
        void setTargetPositions(int a){
            arm.setTargetPosition(a);
        }
    }
    class Claw {
        public CRServo claw = null;
        public Claw(HardwareMap Map) {
            claw = Map.crservo.get("claw");
        }
    }
    class Lift {
        public DcMotor lSlide1 = null;

        public DcMotor lSlide2 = null;
        public Lift(HardwareMap Map) {
            // Beep boop this is the the constructor for the lif
            lSlide1 = Map.dcMotor.get("lSlide1");
            lSlide2 = Map.dcMotor.get("lSlide2");

        }

        public void update() {
            // Beep boop this is the lift update function PID STIJF ISJSIJ Sij
            lSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lSlide1.setPower(1);
            lSlide2.setPower(1);
        }
        void driveInit() {
            lSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lSlide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lSlide1.setDirection(DcMotorSimple.Direction.REVERSE);
            lSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        void setTargetPositions(int a){
            lSlide1.setTargetPosition(a);
            lSlide2.setTargetPosition(a);
        }
    }
//ttgttt


}