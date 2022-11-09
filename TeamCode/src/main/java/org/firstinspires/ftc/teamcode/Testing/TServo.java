package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// lift1     lift2
@TeleOp(name="Servo Test")
public class TServo extends LinearOpMode{
    //general Vars
    private ElapsedTime runtime = new ElapsedTime();

    //servo Vars
    private Servo MS = null;

    @Override
    public void runOpMode() {

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.a){
                MS.setPosition(0);
            }
            if (gamepad1.b){
                MS.setPosition(1);
            }
        }
    }
}
