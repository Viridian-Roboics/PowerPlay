package org.firstinspires.ftc.teamcode.Teliops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Set;

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
    private static double CAP = .5;

    //lift Vars
    private DcMotor L1 = null;
    private double Lspeed = 1;
    private double LMin = 0;
    private double LMax = 0;
    private int LTarget = 0;
    private int BottomLift = 100;
    private int MiddleLift = 200;
    private int TopLift = 300;
    private int ConeLift = 50;
    private boolean SetPos = false;

    //servo Vars
    private Servo LServo = null;

    @Override
    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        L1 = hardwareMap.get(DcMotor.class, "lift1");
        LServo = hardwareMap.get(Servo.class, "LServo");

        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);

        L1.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double FLP = axial + lateral + yaw;
            double FRP = axial - lateral - yaw;
            double BLP = axial - lateral + yaw;
            double BRP = axial + lateral - yaw;

            max = Math.max(Math.abs(FLP), Math.abs(FRP));
            max = Math.max(max, Math.abs(BLP));
            max = Math.max(max, Math.abs(BLP));

            if (max > CAP) {
                FLP = FLP / max * CAP;
                FRP = FLP / max * CAP;
                BLP = max;
                BRP = max;
            }

            //lift
            if (gamepad1.right_bumper) {
                RegMoveLift(1, "Going Up");
            } else if (gamepad1.left_bumper) {
                RegMoveLift(-1, "Going Down");
            } else if (LTarget == 0 || LTarget == L1.getCurrentPosition()) {
                L1.setPower(0);
            }

            if (gamepad1.dpad_up && L1.getCurrentPosition() != TopLift) {
                LTarget = MoveLift(TopLift);
            } else if (gamepad1.dpad_left && L1.getCurrentPosition() != MiddleLift) {
                LTarget = MoveLift(MiddleLift);
            } else if (gamepad1.dpad_down && L1.getCurrentPosition() != BottomLift) {
                LTarget = MoveLift(BottomLift);
            } else if (gamepad1.dpad_right && L1.getCurrentPosition() != ConeLift) {
                LTarget = MoveLift(ConeLift);
            }
            if (LTarget != 0) {
                L1.setPower(Lspeed);
            }

            //claw
            if (gamepad1.a){
                LServo.setPosition(0);
            }
            else if (gamepad1.b){
                LServo.setPosition(1);
            }

            FL.setPower(FLP);
            FR.setPower(FRP);
            BL.setPower(BLP);
            BR.setPower(BRP);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", FLP, FRP);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", BLP, BRP);
            telemetry.addData("Lift Pos", "%4.2f", L1.getCurrentPosition());
            telemetry.update();
        }
    }

    private int MoveLift(int GoalPos){
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L1.setTargetPosition(GoalPos);
        return GoalPos;
    }

    private void RegMoveLift(int down, String status){
        LTarget = 0;
        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L1.setPower(Lspeed * down);
        telemetry.addData("Status", status);
    }
}
