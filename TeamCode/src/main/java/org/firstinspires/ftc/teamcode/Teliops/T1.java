package org.firstinspires.ftc.teamcode.Teliops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private DcMotor L2 = null;
    private double Lspeed = 1;
    private double LMin = 0;
    private double LMax = 0;
    private int BottomLift = 100;
    private int MiddleLift = 200;
    private int TopLift = 300;

    //servo Vars
    private Servo LServo = null;

    @Override
    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        L1 = hardwareMap.get(DcMotor.class, "lift1");
        L2 = hardwareMap.get(DcMotor.class, "lift2");
        LServo = hardwareMap.get(Servo.class, "LServo");

        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);

        L1.setDirection(DcMotor.Direction.REVERSE);
        L2.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double FLP  = axial + lateral + yaw;
            double FRP = axial - lateral - yaw;
            double BLP   = axial - lateral + yaw;
            double BRP  = axial + lateral - yaw;

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
            if (gamepad1.right_bumper){
                L1.setPower(Lspeed);
                L2.setPower(Lspeed);
                telemetry.addData("Status", "Going up");
            }
            else if (gamepad1.left_bumper){
                L1.setPower(-Lspeed);
                L2.setPower(-Lspeed);
                telemetry.addData("Status", "Going down");
            }
            else{
                L1.setPower(0);
                L2.setPower(0);
            }

            if(gamepad1.dpad_up && L1.getCurrentPosition() <= TopLift){
                L1.setTargetPosition(TopLift);
                L2.setTargetPosition(TopLift);
            }
            else if(gamepad1.dpad_left && L1.getCurrentPosition() <= MiddleLift){
                L1.setTargetPosition(MiddleLift);
                L2.setTargetPosition(MiddleLift);
            }
            else if(gamepad1.dpad_down && L1.getCurrentPosition() <= BottomLift){
                L1.setTargetPosition(BottomLift);
                L2.setTargetPosition(BottomLift);
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
}
