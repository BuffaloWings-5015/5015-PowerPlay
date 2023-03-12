package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RegionalTeleop.PoleCenteringState.horizontal;
import static org.firstinspires.ftc.teamcode.RegionalTeleop.PoleCenteringState.notMoving;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MotionLibrary.movement.MecanumDriveEncoders;
import org.firstinspires.ftc.teamcode.MotionLibrary.util.PID;
import org.firstinspires.ftc.teamcode.MotionLibrary.util.Pose2D;
import org.firstinspires.ftc.teamcode.MotionLibrary.util.Vector2D;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="FieldCentricTeleop")
public class RegionalTeleop extends LinearOpMode {


    Definitions robot;
    //Slide Controls
    double targetSlide;
    double targetV4b;
    double fbConst;
    double v4bAdd;
    double v4bAddMulti = 0.1;
    double lSlideConst = 0;
    double lslidePower;
    double v4bPower;
    boolean lSlideMoving = false;
    boolean v4bMoving = false;
    boolean gamepad2LeftLast = true;
    boolean gamepad2UpLast = true;
    boolean gamepad2RightLast = true;
    boolean gamepad2DownLast = true;
    final double v4bTicsToDegrees = 1425.1 * 1.3 / 360;
    double v4bAngle;
    PID v4bPID;
    PID lSLidesPID;

    MecanumDriveEncoders motion;

    PolePipeline detector = new PolePipeline(telemetry);
    double poleCoordinate = detector.polePos();



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status :", "RunOpMode");

        // Declare our motors
        // Make sure your ID's match your configuration
        robot = new Definitions();
        motion = new MecanumDriveEncoders(this, 0);
        robot.robotHardwareMapInit(hardwareMap);
        robot.driveInit();
        robot.lSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lSlide1.setTargetPosition(0);
        robot.lSlide2.setTargetPosition(0);
        //PID stuffs

       // PID.setConstantsSlides(-0.001, 0.0000, 0.0000);
        PID.setConstantsArm(0.0007,0.0000001,0.00001);

        v4bPID = new PID(PID.Type.arm);
       // lSLidesPID = new PID(PID.Type.slides);
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests


        // Retrieve the IMU from the hardware map
        BNO055IMU imu;
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        //imu.initialize(parameters);

        //Camera Init stuffs
        PolePipeline detector = new PolePipeline(telemetry);
        robot.webcam1.setPipeline(detector);

        robot.webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                                @Override
                                                public void onOpened()
                                                {

                                                    telemetry.addData("status:", "not crying");
                                                    robot.webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                                                }

                                                @Override
                                                public void onError(int errorCode)
                                                {
                                                    //Cry about it
                                                    telemetry.addData("status:", "crying");
                                                    }
        });



        //Gripper Controls
        double servoPower = 0;
        double timeSinceServoStateChange = 0;
        servoState lastServoState = servoState.closed;
        servoState currentServoState = servoState.closed;
        boolean servoAuto = false;
        boolean colorSensorLast = false;
        boolean colorSensorCurrent = false;
        double colorSensorCooldown = 0;



        fbConst = 0;
        targetSlide = 0;
        targetV4b = 0;

        robot.v4bar1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lSlide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.v4bar1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        double speedMultiplier = 1;
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //Pole auto align
            if (gamepad1.a) poleState = horizontal;
            else if (gamepad1.b) poleState = notMoving;

            //centerAlongPole();
            switch (poleState) {
                case notMoving:
                    break;
                case horizontal:
                    poleCoordinate = detector.polePos();
                    if (290 < poleCoordinate && poleCoordinate > 310 ) {
                        poleState = PoleCenteringState.linear;
                        break;
                    }

                    moveVector = new Vector2D(-(detector.polePos() - 0) * 0.0005, 0);
                    break;

                case linear:
                    poleDistance = robot.distanceSensor.getDistance(DistanceUnit.CM);

                    if (poleDistance > 11 && poleDistance < 13) {
                        poleState = notMoving;
                        break;
                    }

                    moveVector = new Vector2D(0, (poleDistance - 12) * 0.01);
                    break;
            }


            //Movement
            speedMultiplier = 1.25 - gamepad1.right_trigger;
            if(speedMultiplier > 1) speedMultiplier = 1;

            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;

            centerAlongPole();

            if (poleState != notMoving) {
                if (x != 0 && y !=0) {
                    moveVector = new Vector2D(x,y);
                    moveVector.rotateRadians(robot.imu.getAngularOrientation().firstAngle);
                }
            } else {
                moveVector = new Vector2D(x,y);
                moveVector.rotateRadians(robot.imu.getAngularOrientation().firstAngle);
            }

            moveVector.mult(speedMultiplier);
            moveVector.capAt1();

            motion.move(new Pose2D(moveVector, -gamepad1.right_stick_x));

            //Servo Controller
            if (gamepad1.dpad_up) {
                currentServoState = servoState.opening;
                servoAuto = true;
            } else if (gamepad1.dpad_down) {
                currentServoState = servoState.closed;
                servoAuto = true;
            } else if (gamepad1.dpad_left) {
                currentServoState = servoState.none;
                servoAuto = false;
            }

            if(gamepad2.right_bumper || gamepad2.left_bumper) {
               currentServoState = servoState.none;
               servoAuto = false;
            }

            colorSensorCurrent = robot.colorsensor.alpha() <= 2;

            if (servoAuto) {
                if (colorSensorCurrent && colorSensorLast) {
                    colorSensorCooldown = System.currentTimeMillis();
                }
                if (colorSensorCurrent && System.currentTimeMillis() - colorSensorCooldown >= 250) {
                    currentServoState = servoState.closed;
                }
                if (colorSensorCurrent && !colorSensorLast) {
                    targetSlide += 250;
                   // lSLidesPID.reset();
                }
            }


            //4 bar and slides control
                /*
            float v4barspeed;
            v4barspeed = 1 - gamepad2.right_trigger;
            {
                if (gamepad2.left_stick_y != 0) {
                    robot.v4bar1.setPower(gamepad2.left_stick_y);
                }
                else {
                    robot.v4bar1.setPower(0);
                }
                telemetry.update();
            }*/

            //just p no i or d. (i got i gpu on streets last night btw)

            if (gamepad2.dpad_left){
                robot.v4bar1.setTargetPosition(1150);
                targetSlide = 0;
                targetV4b = 1150;
                if (!gamepad2LeftLast) {
                    newHeightStuffs();
                }
                gamepad2LeftLast = true;
            } else gamepad2LeftLast = false;
            if (gamepad2.dpad_up){
                robot.v4bar1.setTargetPosition(1150);
                targetSlide = 1350;
                targetV4b = 1150;
                if (!gamepad2UpLast) {
                    newHeightStuffs();
                }
                gamepad2UpLast = true;
            } else gamepad2UpLast = false;
            if (gamepad2.dpad_right){
                robot.v4bar1.setTargetPosition(1150);
                targetSlide = 400;
                targetV4b = 1150;
                if (!gamepad2RightLast) {
                    newHeightStuffs();
                }
                gamepad2RightLast = true;
            } else gamepad2RightLast = false;
            if (gamepad2.dpad_down) {
                robot.v4bar1.setTargetPosition(0);
                targetSlide = 0;
                targetV4b = 0;
                if (!gamepad2DownLast) {
                    newHeightStuffs();
                }
                gamepad2DownLast = true;
            } else gamepad2DownLast = false;

            targetSlide += gamepad2.left_stick_y;
            targetV4b += gamepad2.right_stick_y * -8;


            final double KslideMulti;
            final double KV4bMulti;
            KslideMulti = -0.025;
            KV4bMulti = -0.001;



            //Servo Controller
            switch (currentServoState) {
                case closed:
                    servoPower = 0.5;
                    break;
                case opening:
                    if (lastServoState != servoState.opening) {
                        timeSinceServoStateChange = System.currentTimeMillis();
                    } else {
                        if (System.currentTimeMillis() - timeSinceServoStateChange <= 125) {
                            servoPower = -0.5;
                        } else {
                            currentServoState = servoState.none;
                        }
                    }
                    break;
                case none:
                    if (gamepad2.right_bumper) {
                        servoPower = 1;
                    } else if (gamepad2.left_bumper) {
                        servoPower = -1;
                    } else {
                        servoPower = 0;
                    }
                    break;
            }

            robot.claw.setPower(servoPower);

            /*
            robot.lSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.v4bar1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lSlide1.setPower(1);
            robot.lSlide2.setPower(1);
            robot.v4bar1.setPower(0.6);
             */

            //slide and v4bControls
            /*
            double lslidesError = targetSlide-(robot.lSlide1.getCurrentPosition() + robot.lSlide2.getCurrentPosition())/2;

            lSLidesPID.pid(lslidesError);
            lslidePower = lSLidesPID.pidout + lSlideConst;

            robot.lSlide1.setPower(Range.clip(lslidePower, -1, 1));
            robot.lSlide2.setPower(Range.clip(lslidePower, -1, 1));
*/
            double v4bError = targetV4b-robot.v4bar1.getCurrentPosition();

            v4bAngle = robot.v4bar1.getCurrentPosition()/v4bTicsToDegrees + 12; //The 12 is the starting angle of the 4bar (It is a little bit forward)
            v4bAdd = Math.sin(Math.toRadians(v4bAngle)) * v4bAddMulti;
            v4bPID.pid(v4bError);
            v4bPower = v4bPID.pidout + v4bAdd;

            robot.v4bar1.setPower(Range.clip(v4bPower, -1, 1));
            robot.lSlide1.setTargetPosition((int) targetSlide);
            robot.lSlide2.setTargetPosition((int) targetSlide);
            robot.lSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lSlide1.setPower(1);
            robot.lSlide2.setPower(1);

            //telemetry thingies
            telemetry.addData("linear slide encoder", robot.lSlide2.getCurrentPosition());
            telemetry.addData("V4Bar encoder", robot.v4bar1.getCurrentPosition());
            telemetry.addData("linear slide encoder target", targetSlide);
            telemetry.addData("V4Bar encoder target", targetV4b);
                /*
                telemetry.addData("Dpad up", gamepad2.dpad_up);
                telemetry.addData("Dpad right", gamepad2.dpad_right);
                telemetry.addData("Dpad left", gamepad2.dpad_left);
                telemetry.addData("Dpad down", gamepad2.dpad_down);
                 */
            telemetry.addData("Color Sensor alpha", robot.colorsensor.alpha());
            telemetry.addData("Servo State", currentServoState);
            telemetry.addData("v4bar Power", v4bPower);
            telemetry.addData("v4bar Angle", v4bAngle);
            telemetry.addData("v4bar PID out", v4bPID.pidout);
            telemetry.addData("arm p", v4bPID.p);
            telemetry.addData("arm i", v4bPID.i);
            telemetry.addData("arm d", v4bPID.d);
            telemetry.addData("armKP", PID.KPArm);
            telemetry.addData("v4b error", v4bError);
            telemetry.addData("Slides Power", lslidePower);
            //telemetry.addData("slides p", lSLidesPID.p);
          //  telemetry.addData("slides i", lSLidesPID.i);
            //telemetry.addData("slides d", lSLidesPID.d);
           // telemetry.addData("slides error", lslidesError);
            telemetry.addData("Heading", robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("Pole Coordinate", poleCoordinate);
            telemetry.addData("Pole Distance", poleDistance);
            telemetry.addData("pole pos enum", detector.polePos());
            telemetry.update();

                //used for servo control
            lastServoState = currentServoState;
            colorSensorLast = colorSensorCurrent;
        }



    }
//ttgttt

    PoleCenteringState poleState = notMoving;
    Vector2D moveVector = new Vector2D();

    double poleDistance = 0;
    public void centerAlongPole() {
        switch (poleState) {
            case notMoving:
                break;
            case horizontal:
                poleCoordinate = detector.polePos();
                if (280 < poleCoordinate && poleCoordinate > 320 ) {
                    poleState = PoleCenteringState.linear;
                    break;
                }

                moveVector = new Vector2D(-(detector.polePos() - 0) * 0.001, 0);
                break;

            case linear:
                poleDistance = robot.distanceSensor.getDistance(DistanceUnit.CM);

                if (poleDistance > 11 && poleDistance < 13) {
                    poleState = notMoving;
                    break;
                }

                moveVector = new Vector2D(0, (poleDistance - 12) * 0.05);
                break;
        }



    }
    public enum servoState {
        opening,
        closed,
        none
    }

    public enum PoleCenteringState {
        notMoving,
        horizontal,
        linear
    }
    public void newHeightStuffs() {
        v4bMoving = true;
       // lSlideMoving = true;
       // lSLidesPID.reset();
        v4bPID.reset();
    }
}