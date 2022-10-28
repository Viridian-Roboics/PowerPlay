package org.firstinspires.ftc.teamcode.Teliops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Main Teliop")
public class T1 extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;

    @Override
    public void runOpMode() {

        FL  = hardwareMap.get(DcMotor.class, "FL");
        FR  = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);

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
