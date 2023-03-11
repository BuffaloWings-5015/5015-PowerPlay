package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

/**
 * This is NOT an opmode.
 * <p>
 * This class is used to define all the specific hardware for a single robot.
 * In this case that robot is a Mecanum driven Robot.
 * Mecanum robots utilize 4 motor driven wheels.
 * <p>
 * <p>
 * <p>
 * Software 1 min and 45 second Elevator Speech:
 * - Github:
 * - We utilize an open source coding platform known as github. Github adds value to our team in 3 ways:
 * - Version Control:
 * - Allows us to keep track of multiple files, backup our code, and share it with the world
 * - Colaboration:
 * - Allow us to make our code open source to the interwebs.
 * - Backup:
 * - We will never lose a file again
 * - Computer vision
 * - Vuforia:
 * - Definition File
 * - Auto
 * - Teleop
 * - Past failures
 * - Tensorflow
 */
public class Definitions {
    public static final String VUFORIA_KEY = "AQFyZOr/////AAABmRL+3QMh6kiBj2OqwKGebApeLvS635fqPcuCrcT8QdD0u4Y4EtbuBcQx1GtwkPSykB4xBRu+ZM5NapeDwTwkVQdSVNjorl1ebJalwSv0gJcSUVDZy/S45PyYVMuwvl6hdI1sFTKwnejvz8eyyxEpaAv4FZCP99BakBW7reGXUYYIHyXgsBDFOpprXd8Ka0GmgHYixugvl9WkV3DK0f4H3TG0d93QR6uY9Yp7Iyr01XVJ+Oym7YRKEzEywe3O9HzOIA5j/fh3zTg9GSpWpdXq400rxgviEpncr3YnynCzm9PjwPy9K8rfROJz5/2ZcO8uWcjxCCdPLbreVKIeKrpqBtsGtNEr8X2dFNhwLYfq8cKr";
    public DcMotor leftFront = null;
    public BNO055IMU imu;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor v4bar1 = null;
    public DcMotor lSlide1 = null;

    public DcMotor lSlide2 = null;
    public CRServo claw = null;
    public final double ticks_in_degree = 1421.1 / 360.0 * 1.3;
    //port 2 on the thing (ic2)
    public ColorSensor colorsensor = null;
    public OpenCvCamera webcam1, webcam2;
    public DistanceSensor distanceSensor;


    double speedMultiplier = 1;







    final float mmPerInch = 25.4f;
    // Class Members
    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */


    final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);



    public void robotHardwareMapInit(HardwareMap Map) {
        //telemetry.addData("Status :", "hardware map");
        leftFront = Map.dcMotor.get("leftFront");
        rightFront = Map.dcMotor.get("rightFront");
        leftBack = Map.dcMotor.get("leftRear");
        rightBack = Map.dcMotor.get("rightRear");

        v4bar1 = Map.get(DcMotorEx.class, "v4bar");
        lSlide1 = Map.dcMotor.get("lSlide1");
        lSlide2 = Map.dcMotor.get("lSlide2");
        claw = Map.crservo.get("claw");
        imu = Map.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        colorsensor = Map.colorSensor.get("colorSensor");
        distanceSensor = Map.get(DistanceSensor.class, "distanceSensor");


        int cameraMonitorViewId = Map.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", Map.appContext.getPackageName());

        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(Map.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //webcam2 = OpenCvCameraFactory.getInstance().createWebcam(Map.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
    }

    void driveInit() {

        //telemetry.addData("Status :", "init");
        //Stop and reset motor encoders to ensure consistent values
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        v4bar1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets the motors to run through driver input instead of running to an encoder position
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        v4bar1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSlide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Sets each motor to hold its current position while having zero power set
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        v4bar1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //This sets the robot to drive straight by default
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        lSlide1.setDirection(DcMotorSimple.Direction.REVERSE);
        lSlide2.setDirection(DcMotorSimple.Direction.FORWARD);



    }

    void autoInit() {

    }
public void rotate4bar(double degrees, double power){

    }


}