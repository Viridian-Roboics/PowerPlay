package org.firstinspires.ftc.teamcode.Teliops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

// lift1     lift2
@TeleOp(name="Main Teliop")
public class T1 extends LinearOpMode{
    //general Vars
    private ElapsedTime runtime = new ElapsedTime();

    //drive Vars
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;

    //lift Vars
    private DcMotor L1 = null;
    private DcMotor L2 = null;
    private double Lspeed = .25;

    @Override
    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        L1 = hardwareMap.get(DcMotor.class, "lift1");
        L2 = hardwareMap.get(DcMotor.class, "lift2");

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);

        L1.setDirection(DcMotor.Direction.REVERSE);
        L2.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double FLP  = axial + lateral + yaw;
            double FRP = axial - lateral - yaw;
            double BLP   = axial - lateral + yaw;
            double BRP  = axial + lateral - yaw;

            max = Math.max(Math.abs(FLP), Math.abs(FRP));
            max = Math.max(max, Math.abs(BLP));
            max = Math.max(max, Math.abs(BLP));

            if (max > 1.0) {
                FLP  /= max;
                FRP /= max;
                BLP   /= max;
                BRP  /= max;
            }

            if (gamepad1.right_bumper){
                L1.setPower(Lspeed);
                L1.setPower(Lspeed);
                telemetry.addData("Status", "Going up");
            }
            else if (gamepad1.left_bumper){
                L1.setPower(-Lspeed);
                L1.setPower(-Lspeed);
                telemetry.addData("Status", "Going down");
            }
            else{
                L1.setPower(0);
                L1.setPower(0);
            }

            FL.setPower(FLP);
            FR.setPower(FRP);
            BL.setPower(BLP);
            BR.setPower(BRP);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", FLP, FRP);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", BLP, BRP);
            telemetry.update();
        }
    }
}
