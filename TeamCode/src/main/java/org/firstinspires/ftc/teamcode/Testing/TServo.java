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
    private Servo LServo = null;
    private Servo PickServo = null;

    @Override
    public void runOpMode() {
        LServo = hardwareMap.get(Servo.class, "LServo");
        PickServo = hardwareMap.get(Servo.class, "PickServo");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.a){
                LServo.setPosition(0);
            }
            if (gamepad1.b){
                LServo.setPosition(1);
            }
            if (gamepad1.y){
                PickServo.setPosition(0);
            }
            if (gamepad1.x){
                PickServo.setPosition(1);
            }
        }
    }
}
