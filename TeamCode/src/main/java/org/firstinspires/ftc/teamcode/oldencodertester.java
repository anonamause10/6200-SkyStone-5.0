package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by ianso on 7/23/2019.
 */
@TeleOp(name="glunk", group="Iterative Opmode")
public class oldencodertester extends LinearOpMode{
    // Declare OpMode members.

    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor UD = null;
    private Servo thatYes = null;
    private int target = 0;
    private int x = 0;
    private double slowspeed = 0.3;
    private boolean xPrev = false;
    private boolean aPrev = false;
    private boolean bPrev = false;
    private boolean lbPrev = false;
    private boolean rbPrev = false;
    private boolean upD = false;
    private boolean downD = false;
    private boolean leftD = false;
    private boolean rightD = false;
    private double ticksperinches = 27.8521150411;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.get(DcMotor.class, "fL");
        FR = hardwareMap.get(DcMotor.class, "fR");
        BL = hardwareMap.get(DcMotor.class, "bL");
        BR = hardwareMap.get(DcMotor.class, "bR");
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setTargetPosition((int)(target * ticksperinches));
        BL.setTargetPosition((int)(target * ticksperinches));
        BR.setTargetPosition((int)(target * ticksperinches));
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        UD = hardwareMap.get(DcMotor.class, "uD");
        UD.setDirection(DcMotor.Direction.FORWARD);
        UD.setPower(0);
        thatYes = hardwareMap.get(Servo.class, "servo");
        thatYes.setPosition(.4);
        telemetry.addData("Robot", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double speed = 0;
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while (opModeIsActive()) {

            speed = slowspeed;
            if(Math.abs(FR.getTargetPosition() - FR.getCurrentPosition()) < 100 && x<2)
                speed = 0.15 ;
            if(!FR.isBusy()) {
                speed = 0;
                FL.setTargetPosition(0);
                FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }else{
                FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            FR.setPower(speed);
            BL.setPower(speed);
            BR.setPower(speed);
            if(x < 2) {
                double power = FR.getTargetPosition()!=0?(FR.getTargetPosition() - FR.getCurrentPosition())/(double)FR.getTargetPosition():0;
                FL.setPower(speed*(x%2==0?power:-1*power));
                if(FR.getTargetPosition()==0){
                    telemetry.addData("Droonk",", very big droonk");
                }
            }else if (x < 4){
                FL.setPower(BR.getPower());
            }


            if (gamepad1.x&&!xPrev) {

            }
            xPrev = gamepad1.x;
            if (gamepad1.dpad_down && !downD){
                driveBackward(speed);
            }else if(gamepad1.dpad_up && !upD){
                driveForward(speed);
            }else if(gamepad1.dpad_left && !leftD){
                driveLeft(speed);
            }else if(gamepad1.dpad_right && !rightD){
                driveRight(speed);
            }
            if(gamepad1.a&&!aPrev){
                target++;
            }else if(gamepad1.b&&!bPrev){
                target--;
            }

            if(gamepad1.right_bumper&&!rbPrev){
                slowspeed+=0.025;
            }else if(gamepad1.left_bumper&&!lbPrev){
                slowspeed-=0.025;
            }

            if(gamepad1.right_trigger==1.0){
                slowspeed+=0.025;
            }else if(gamepad1.left_bumper&&!lbPrev){
                slowspeed-=0.025;
            }
            aPrev = gamepad1.a;
            bPrev = gamepad1.b;
            downD = gamepad1.dpad_down;
            rbPrev=gamepad1.right_bumper;
            lbPrev=gamepad1.left_bumper;
            upD = gamepad1.dpad_up;
            leftD = gamepad1.dpad_left;
            rightD = gamepad1.dpad_right;

            telemetry.addData("Wheel Power", "front left (%.2f), front right (%.2f), " +
                            "back left (%.2f), back right (%.2f)", FL.getPower(), FR.getPower(),
                    BL.getPower(), BR.getPower());
            telemetry.addData("Wheel Position", "front left (%.1f), front right (%.1f), " +
                            "back left (%.1f), back right (%.1f)", (float)FL.getCurrentPosition(), (float)FR.getCurrentPosition(),
                    (float)BL.getCurrentPosition(), (float)BR.getCurrentPosition());
            telemetry.addData("Target", target + " in. ||" + (int)(target * ticksperinches) + " ticks");
            telemetry.addData("Speed", ": " + speed);
            telemetry.addData("Slowing down Speed", ": " + slowspeed);

            telemetry.update();
        }
    }
    private void driveForward(double speed){
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setTargetPosition((int)(target * ticksperinches));
        BL.setTargetPosition((int)(target * ticksperinches));
        BR.setTargetPosition((int)(target * ticksperinches));
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        x = 0;
    }
    private void driveBackward(double speed){
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setTargetPosition((int)(-target * ticksperinches));
        BL.setTargetPosition((int)(-target * ticksperinches));
        BR.setTargetPosition((int)(-target * ticksperinches));
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        x = 1;
    }
    private void driveLeft(double speed){
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setTargetPosition((int)(target * ticksperinches));
        BL.setTargetPosition((int)(target * ticksperinches));
        BR.setTargetPosition((int)(-target * ticksperinches));
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        x = 2;
    }
    private void driveRight(double speed){
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setTargetPosition((int)(-target * ticksperinches));
        BL.setTargetPosition((int)(-target * ticksperinches));
        BR.setTargetPosition((int)(target * ticksperinches));
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        x = 3;
    }
}

