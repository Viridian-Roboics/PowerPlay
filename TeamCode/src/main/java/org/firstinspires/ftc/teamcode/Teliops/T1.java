package org.firstinspires.ftc.teamcode.Teliops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


// input buttons leftover: x
// lift1     lift2
@TeleOp(name="Teliop One")
public class T1 extends LinearOpMode{
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
    private boolean Pickopen = false;
    private boolean Lopen = false;
        //picker drop
    private double TopPick = 0;
    private double BottomPick = 1;
        //main grabber
    private double TopL = 0;
    private double BottomL = .9;
    private int ClawBlock = 0;
    private int PickBlock = 0;

    @Override
    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        L1 = hardwareMap.get(DcMotor.class, "lift1");
        LServo = hardwareMap.get(Servo.class, "LServo");
        PickServo = hardwareMap.get(Servo.class, "PickServo");

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
        LMax = LMin + 2762;
        BottomLift = LMin + 1490;
        MiddleLift = LMin + 2279;
        TopLift = LMin + 2762;
        ConeLift = LMin + 1000;


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

            if (gamepad1.right_trigger > .25 || gamepad2.right_trigger > .25){
                CAP = .25;
            }
            else if (gamepad1.left_trigger > .25 || gamepad2.left_trigger > .25){
                CAP = 1;
            }
            else{
                CAP = .5;
            }

            if (max > CAP) {
                FLP = FLP / max * CAP;
                FRP = FRP / max * CAP;
                BLP = BLP / max * CAP;
                BRP = BRP / max * CAP;
            }

            //lift
            if ((gamepad1.right_bumper || gamepad2.right_bumper) && (L1.getCurrentPosition() <= LMax || IgnoreLift)) {
                RegMoveLift(1, "Going Up", ULspeed);
            } else if((gamepad1.left_bumper || gamepad2.left_bumper) && (L1.getCurrentPosition() >= LMin || IgnoreLift)) {
                RegMoveLift(-1, "Going Down", DLspeed);
            } else if (LTarget == 0 || LTarget == L1.getCurrentPosition()) {
                L1.setPower(0.005);
            }

            if ((gamepad1.dpad_up || gamepad2.dpad_up) && L1.getCurrentPosition() != TopLift) {
                LTarget = MoveLift(TopLift);
                PickServo.setPosition(TopPick);
            } else if ((gamepad1.dpad_left  || gamepad2.dpad_left) && L1.getCurrentPosition() != MiddleLift) {
                LTarget = MoveLift(MiddleLift);
            } else if ((gamepad1.dpad_down || gamepad2.dpad_down) && L1.getCurrentPosition() != BottomLift) {
                LTarget = MoveLift(BottomLift);
            } else if ((gamepad1.dpad_right || gamepad2.dpad_right) && L1.getCurrentPosition() != ConeLift) {
                LTarget = MoveLift(ConeLift);
            }

            if (LTarget != 0) {
                if (L1.getCurrentPosition() > LTarget){
                    L1.setPower(DLspeed);
                } else{
                    L1.setPower(ULspeed);
                }
            }

            //claw
            if (ClawBlock == 0) {
                if ((gamepad1.a || gamepad2.a) && Lopen) {
                    sleep(500);
                    Lopen = false;
                    LServo.setPosition(TopL);
                } else if ((gamepad1.a || gamepad2.a) && !Lopen) {
                    sleep(500);
                    Lopen = true;
                    LServo.setPosition(BottomL);
                }
            } else if(ClawBlock >= 20000){
                ClawBlock = 0;
            } else{
                ClawBlock += 1;
            }

            //pick lower
            if (PickBlock == 0) {
                if ((gamepad1.b || gamepad2.b) && Pickopen) {
                    sleep(500);
                    Pickopen = false;
                    PickServo.setPosition(TopPick);
                } else if ((gamepad1.b || gamepad2.b) && !Pickopen) {
                    sleep(500);
                    Pickopen = true;
                    PickServo.setPosition(BottomPick);
                }
            } else if(PickBlock >= 20000){
                PickBlock = 0;
            } else{
                PickBlock += 1;
            }

            //fuck lift restrictions
            if ((gamepad1.y || gamepad2.y) && IgnoreLift == false){
                IgnoreLift = true;
            } else if ((gamepad1.y || gamepad2.y) && IgnoreLift == true){
                IgnoreLift = false;
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

    private int MoveLift(int GoalPos){
        L1.setTargetPosition(GoalPos);
        L1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L1.setTargetPosition(GoalPos);
        return GoalPos;
    }

    private void RegMoveLift(int down, String status, double speed){
        LTarget = 0;
        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L1.setPower(speed * down);
        telemetry.addData("Status", status);
    }
}
