package org.firstinspires.ftc.teamcode.AutonTest;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// lift1     lift2
@Disabled
@Autonomous(name="pAuton 1")
public class practiceauton extends LinearOpMode{
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

    //gryo
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

    @Override
    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        L1 = hardwareMap.get(DcMotor.class, "lift1");
        L2 = hardwareMap.get(DcMotor.class, "lift2");
        LServo = hardwareMap.get(Servo.class, "LServo");

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);

        L1.setDirection(DcMotor.Direction.REVERSE);
        L2.setDirection(DcMotor.Direction.FORWARD);

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

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Ready to rock and roll");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //movement code
        encoderDrive(.75, 12, false, 10000);
        encoderDrive(.25, -8, true, 10000);

    }

    public void encoderDrive(double speed, double MoveIN, boolean strafe, double timeoutS) {
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
            sleep(100);
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

    public void LiftPosSet(int LiftTo){
        if (LiftTo > L1.getCurrentPosition()) {
            L1.setTargetPosition(LiftTo);
            L2.setTargetPosition(LiftTo);
            L1.setPower(1);
            L2.setPower(1);
        }
        if (L1.isBusy()){

        }
        else{
            L1.setPower(0);
            L2.setPower(0);
        }
    }
}
