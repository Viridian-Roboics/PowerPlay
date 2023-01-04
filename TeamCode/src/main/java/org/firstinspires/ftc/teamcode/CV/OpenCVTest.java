package org.firstinspires.ftc.teamcode.CV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.opencv.objdetect.QRCodeDetector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class OpenCVTest extends LinearOpMode {
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private double CAP = .5;
    private double axial = 0;
    private double lateral = 0;
    private double yaw = 0;

    OpenCvWebcam webcam;
    QRCodeDetector det;
    boolean found;
    String message;

    @Override
    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new SimplePipeline());
        det = new QRCodeDetector();

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                //Cam no open
            }
        });
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            double max;

            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            double FLP = axial + lateral + yaw;
            double FRP = axial - lateral - yaw;
            double BLP = axial - lateral + yaw;
            double BRP = axial + lateral - yaw;
            max = Math.max(Math.max(Math.max(Math.abs(FLP), Math.abs(FRP)), Math.abs(BLP)), Math.abs(BRP));

            if (gamepad1.right_trigger > .25 || gamepad2.right_trigger > .25){CAP = .25;}
            else if (gamepad1.left_trigger > .25 || gamepad2.left_trigger > .25){CAP = 1;}
            else{CAP = .5;}
            if (max > CAP) {FLP = FLP / max * CAP; FRP = FRP / max * CAP; BLP = BLP / max * CAP;BRP = BRP / max * CAP;}

            FL.setPower(FLP);
            FR.setPower(FRP);
            BL.setPower(BLP);
            BR.setPower(BRP);
        }
    }

    public class SimplePipeline extends OpenCvPipeline{
        public Mat processFrame(Mat input){
            String decoded = det.detectAndDecode(input);
            if (decoded.length() > 5){
                telemetry.addData("Decoded",decoded);
                telemetry.update();
                sleep(10000);
                found = true;
                message = decoded;
            }
            return input;
        }

    }
}