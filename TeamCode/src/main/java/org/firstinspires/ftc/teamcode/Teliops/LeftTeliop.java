package org.firstinspires.ftc.teamcode.Teliops;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


// input buttons leftover: x
// lift1     lift2
@TeleOp(name = "LeftTeliop")
public class LeftTeliop extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    //general Vars
    private ElapsedTime runtime = new ElapsedTime();

    //drive Vars
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private double CAP = .5;
    private double axial = 0;
    private double lateral = 0;
    private double yaw = 0;

    //lift Vars
    private DcMotor L1 = null;
    private double ULspeed = 1;
    private double DLspeed = .5;
    private int LMin = 0;
    private int LMax = 0;
    private int LTarget = 0;
    private int BottomLift = 0;
    private int MiddleLift = 0;
    private int TopLift = 0;
    private int ConeLift = 0;
    private boolean IgnoreLift = false;

    //servo Vars
    private Servo LServo = null;
    private Servo PickServo = null;
    private Servo ClawServo = null;
    private boolean Pickopen = false;
    private boolean Lopen = false;
    //picker drop
    private double TopPick = 0;
    private double BottomPick = 1;
    //main grabber
    private double TopL = .1;
    private double BottomL = .3;
    private int ClawBlock = 0;
    private int PickBlock = 0;

    private boolean moved = false;
    private boolean turned = false;

    final double moveSpeed = 0.25;
    final int moveForwardDist = 6;
    final int directionalOffset = 4;
    final int moveSpeedMultiplier = 2;

    @Override
    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        L1 = hardwareMap.get(DcMotor.class, "lift1");
        LServo = hardwareMap.get(Servo.class, "LServo");
        PickServo = hardwareMap.get(Servo.class, "PickServo");
        ClawServo = hardwareMap.get(Servo.class, "clawservo2");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);

        L1.setDirection(DcMotor.Direction.REVERSE);

        LMin = L1.getCurrentPosition();
        LMax = LMin + 3300;
        BottomLift = LMin + 1379;
        MiddleLift = LMin + 2189;
        TopLift = LMin + 2718;
        ConeLift = LMin + 614;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            double FLP = axial + lateral + yaw;
            double FRP = axial - lateral - yaw;
            double BLP = axial - lateral + yaw;
            double BRP = axial + lateral - yaw;

            max = Math.max(Math.abs(FLP), Math.abs(FRP));
            max = Math.max(max, Math.abs(BLP));
            max = Math.max(max, Math.abs(BRP));

            if (gamepad1.right_trigger > .25 || gamepad2.right_trigger > .25) {
                CAP = .25;
            } else if (gamepad1.left_trigger > .25 || gamepad2.left_trigger > .25) {
                CAP = 1;
            } else {
                CAP = .5;
            }

            if (max > CAP) {
                FLP = FLP / max * CAP;
                FRP = FRP / max * CAP;
                BLP = BLP / max * CAP;
                BRP = BRP / max * CAP;
            }

            //lift
            if ((gamepad1.right_bumper || gamepad2.right_bumper) && (L1.getCurrentPosition() <= LMax)) {
                RegMoveLift(1, "Going Up", ULspeed);
            } else if ((gamepad1.left_bumper || gamepad2.left_bumper) && (L1.getCurrentPosition() >= LMin)) {
                RegMoveLift(-1, "Going Down", DLspeed);
            } else if (LTarget == 0 || LTarget == L1.getCurrentPosition()) {
                L1.setPower(0.00);
            }
//            } else if  (L1.getCurrentPosition() == LMax){
//                L1.setPower(00);
//            } else if(L1.getCurrentPosition() == LMin){
//                L1.setPower(0);
//            }
            else if (gamepad1.right_bumper == false || gamepad2.right_bumper == false) {
                L1.setPower(0);
            } else if (gamepad2.left_bumper == false || gamepad2.left_bumper == false) {
                L1.setPower(0);
            }

            if ((gamepad1.dpad_up || gamepad2.dpad_up) && L1.getCurrentPosition() != TopLift) {
                LTarget = MoveLift(TopLift);
                PickServo.setPosition(TopPick);
            } else if ((gamepad1.dpad_left || gamepad2.dpad_left) && L1.getCurrentPosition() != MiddleLift) {
                LTarget = MoveLift(MiddleLift);
            } else if ((gamepad1.dpad_down || gamepad2.dpad_down) && L1.getCurrentPosition() != BottomLift) {
                LTarget = MoveLift(BottomLift);
            } else if ((gamepad1.dpad_right || gamepad2.dpad_right) && L1.getCurrentPosition() != ConeLift) {
                LTarget = MoveLift(ConeLift);
            }

            if (LTarget != 0) {
                if (L1.getCurrentPosition() > LTarget) {
                    L1.setPower(DLspeed);
                } else {
                    L1.setPower(ULspeed);
                }
            }

            //claw
            //if something keeps being weird with claw
            // if (gamepad1.a || gamepad2.a){
            // Lservo.setPosition(TopL);
            // }
            //if (gamepad1.x || gamepad2.x){
            // Lservo.setPosition(BottomL);
            // }
            //
            //
            //

            if (ClawBlock == 0) {
                if ((gamepad1.a || gamepad2.a) && Lopen) {
//                    sleep(500);
                    Lopen = false;
                    LServo.setPosition(TopL);
                } else if ((gamepad1.a || gamepad2.a) && !Lopen) {
//                    sleep(500);
                    Lopen = true;
                    LServo.setPosition(BottomL);
                }
            } else if (ClawBlock >= 20000) {
                ClawBlock = 0;
            } else {
                ClawBlock += 1;
            }

            //pick lower
            if (PickBlock == 0) {
                if ((gamepad1.b || gamepad2.b) && Pickopen) {
                    sleep(500);
                    Pickopen = false;
                    PickServo.setPosition(TopPick);
                    ClawServo.setPosition(BottomPick);
                } else if ((gamepad1.b || gamepad2.b) && !Pickopen) {
                    sleep(500);
                    Pickopen = true;
                    PickServo.setPosition(BottomPick);
                    ClawServo.setPosition(TopPick);
                }
            } else if (PickBlock >= 20000) {
                PickBlock = 0;
            } else {
                PickBlock += 1;
            }


            if ((gamepad1.x || gamepad2.x)) {
                L1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad1.y || gamepad2.y) {
                if (turned) {
                    encoderDrive(moveSpeed*moveSpeedMultiplier, -moveForwardDist*0.5, false, 10000, false);
                    turnDegrees(90);
                    turned = false;
                } else {
                    turnDegrees(-90);
                    encoderDrive(moveSpeed*moveSpeedMultiplier, moveForwardDist*0.5, false, 10000, false);
                    turned = true;
                }
                sleep(500);
            }
            if (gamepad1.left_trigger > 0.8 || gamepad2.right_trigger > 0.8) {
                if (!moved) {
                    encoderDrive(moveSpeed*moveSpeedMultiplier, moveForwardDist, false, 10000, true);
                    encoderDrive(moveSpeed*moveSpeedMultiplier, directionalOffset, true, 10000, true);
                    moved = true;
                } else {
                    encoderDrive(moveSpeed*moveSpeedMultiplier, -directionalOffset, true, 10000, true);
                    encoderDrive(moveSpeed*moveSpeedMultiplier, -moveForwardDist, false, 10000, true);
                    moved = false;
                }
            }


            FL.setPower(FLP);
            FR.setPower(FRP);
            BL.setPower(BLP);
            BR.setPower(BRP);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", FLP, FRP);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", BLP, BRP);
            telemetry.addData("Lift Pos", L1.getCurrentPosition());
            telemetry.addData("ClawBlock: " + ClawBlock, "PickBlock: " + PickBlock);
            telemetry.update();
        }
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
        FL.setPower(Math.abs(moveSpeed*moveSpeedMultiplier));
        FR.setPower(Math.abs(moveSpeed*moveSpeedMultiplier));
        BL.setPower(Math.abs(moveSpeed*moveSpeedMultiplier));
        BR.setPower(Math.abs(moveSpeed*moveSpeedMultiplier));

        while (opModeIsActive() && (runtime.seconds() < 10000) && (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) {
            telemetry.addData("Running to", " %7d", newMoveTarget);
            telemetry.addData("Currently at", " at %7d :%7d :%7d :%7d", FL.getCurrentPosition(), FR.getCurrentPosition(), BL.getCurrentPosition(), BR.getCurrentPosition());
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
    }

    private int MoveLift(int GoalPos) {
        L1.setTargetPosition(GoalPos);
        L1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L1.setTargetPosition(GoalPos);
        return GoalPos;
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
            newMoveTarget = FL.getCurrentPosition() + (int) (MoveIN * COUNTS_PER_INCH);
            if (!strafe) {
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
            } else {
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
                telemetry.addData("Currently at", " at %7d :%7d :%7d :%7d", FL.getCurrentPosition(), FR.getCurrentPosition(), BL.getCurrentPosition(), BR.getCurrentPosition());
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

    private void RegMoveLift(int down, String status, double speed) {
        LTarget = 0;
        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L1.setPower(speed * down);
        telemetry.addData("Status", status);
    }
}
