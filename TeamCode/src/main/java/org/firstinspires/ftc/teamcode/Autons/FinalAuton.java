package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Testing.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.lang.annotation.Target;
import java.util.ArrayList;

@Autonomous(name = "Complex Auton", group = "Robot")
public class FinalAuton extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C270 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 1078.03779;
    double fy = 1084.50988;
    double cx = 580.850545;
    double cy = 245.959325;

    // UNITS ARE METERS
    double tagsize = 0.04;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    //motors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;

    //encoders
    static final double COUNTS_PER_MOTOR_REV = 1440 ;
    static final double DRIVE_GEAR_REDUCTION = 1.0 ;
    static final double WHEEL_DIAMETER_INCHES = 4.0 ;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.7;

    //gry
    private BNO055IMU imu = null;
    private double robotHeading  = 0;
    private double headingOffset = 0;
    private double headingError  = 0;

    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;

    static final double TURN_SPEED = 0.2;
    static final double HEADING_THRESHOLD = .5;
    static final double P_TURN_GAIN = 0.02;
    static final double P_DRIVE_GAIN = 0.03;

    //servo Vars
    private Servo LServo = null;

    //lift Vars
    private DcMotor L1 = null;
    private DcMotor L2 = null;
    private int BottomLift = 100;
    private int MiddleLift = 200;
    private int TopLift = 300;
    private int ConeLift = 50;
    private int Ltarget = 0;

    final double moveSpeed = 0.25;
    final int moveForwardDist = 8;
    final int directionalOffset = 8;

    boolean detected = false;
    int detectedZone = -1;

    void grabConeRoutine() {
        /* Basic (Untested) Routine */

        encoderDrive(0.5, 6, true, 10000, true);
        encoderDrive(0.5, 13, false, 10000, true);
        encoderDrive(0.5, -4, true, 10000, true);
        LiftPosSet(300,.5);
    }

    void turnDegrees(int degrees) {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int degreeRatio = (int) 1000.0 / 120;
        int newMoveTarget = FL.getCurrentPosition() + (degreeRatio * degrees);

        FL.setTargetPosition(-newMoveTarget);
        FR.setTargetPosition(newMoveTarget);
        BL.setTargetPosition(-newMoveTarget);
        BR.setTargetPosition(newMoveTarget);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        FL.setPower(Math.abs(moveSpeed));
        FR.setPower(Math.abs(moveSpeed));
        BL.setPower(Math.abs(moveSpeed));
        BR.setPower(Math.abs(moveSpeed));

        while (opModeIsActive() && (runtime.seconds() < 10000) && (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) {
            telemetry.addData("Running to", " %7d", newMoveTarget);
            telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d", FL.getCurrentPosition(), FR.getCurrentPosition(), BL.getCurrentPosition(), BR.getCurrentPosition());
            telemetry.update();
        }

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(100);
    }


    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        L1 = hardwareMap.get(DcMotor.class, "lift1");
        LServo = hardwareMap.get(Servo.class, "LServo");

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);

        L1.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        resetHeading();

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }

        });

        waitForStart();

        telemetry.setMsTransmissionInterval(50);

        while (opModeIsActive()) {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if (detections != null) {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if (detected) {
                    telemetry.addData("Status", "Already detected, stopping");
                }
                else if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for (AprilTagDetection detection : detections) {
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));

                        int detectedId = detection.id;
                        detected = true;
                        detectedZone = detectedId;

                        // ignore warnings, they'll disappear when you add motor movement
                        switch (detectedId) {
                            case 1: {
                                grabConeRoutine();
                                //encoderDrive(moveSpeed, moveForwardDist, false, 1000, true);
                                //encoderDrive(moveSpeed, -directionalOffset,true, 1000, true);
                                // case 1
                                break;
                            }
                            case 2: {
                                grabConeRoutine();
                                //encoderDrive(moveSpeed, moveForwardDist, false, 10000, true);
                                // case 2
                                break;
                            }
                            case 3: {
                                grabConeRoutine();
                                //encoderDrive(moveSpeed, moveForwardDist, false, 10000, true);
                                //encoderDrive(moveSpeed, directionalOffset, true, 10000, true);
                                // case 3
                                break;
                            }
                        }
                    }
                }

                telemetry.update();
            }

            sleep(20);
        }
    }

    public void encoderDrive(double speed, double MoveIN, boolean strafe, double timeoutS, boolean sleep) {
        int newMoveTarget;

        if (opModeIsActive()) {
            runtime.reset();
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            newMoveTarget = FL.getCurrentPosition() + (int)(MoveIN * COUNTS_PER_INCH);
            if (!strafe){
                FL.setTargetPosition(newMoveTarget);
                FR.setTargetPosition(newMoveTarget);
                BL.setTargetPosition(newMoveTarget);
                BR.setTargetPosition(newMoveTarget);

                FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                runtime.reset();
                FL.setPower(Math.abs(speed));
                FR.setPower(Math.abs(speed));
                BL.setPower(Math.abs(speed));
                BR.setPower(Math.abs(speed));
            }
            else{
                FL.setTargetPosition(newMoveTarget);
                FR.setTargetPosition(-newMoveTarget);
                BL.setTargetPosition(-newMoveTarget);
                BR.setTargetPosition(newMoveTarget);

                FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                runtime.reset();
                FL.setPower(Math.abs(speed));
                FR.setPower(Math.abs(speed));
                BL.setPower(Math.abs(speed));
                BR.setPower(Math.abs(speed));
            }

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) {
                telemetry.addData("Running to", " %7d", newMoveTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d", FL.getCurrentPosition(), FR.getCurrentPosition(), BL.getCurrentPosition(), BR.getCurrentPosition());
                telemetry.update();
            }

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (sleep) {
                sleep(100);
            }
        }
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0, turnSpeed);
        }

        moveRobot(0, 0);
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed  = turn;

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        FL.setPower(leftSpeed);
        FR.setPower(rightSpeed);
        BL.setPower(leftSpeed);
        BR.setPower(rightSpeed);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;
        robotHeading = getRawHeading() - headingOffset;
        headingError = targetHeading - robotHeading;

        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetHeading() {
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    public void LiftPosSet(int LiftTo, double speed){

        L1.setTargetPosition(LiftTo);
        L1.setPower(1);
        runtime.reset();
        L1.setPower(Math.abs(speed));
    }

}
