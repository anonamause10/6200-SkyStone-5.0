package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.ImageTargetBuilder;

/**
 * Created by ianso on 7/22/2019.
 * ENCODER TICKS PER INCH:
 */
@TeleOp(name="EncoderTester")
public class EncoderTester extends LinearOpMode{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor UD = null;
    private Servo thatYes = null;
    private int[] targets = new int[4];
    private int x = 0;
    private boolean xPrev = false;
    private boolean aPrev = false;
    private boolean bPrev = false;
    private boolean upD = false;
    private boolean downD = false;
    private boolean active = false;
    private boolean reset = false;
    private int[] threeTargets = new int[12];
    private int modi = 0;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    //ree
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
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setTargetPosition(targets[0]);
        FR.setTargetPosition(targets[1]);
        BL.setTargetPosition(targets[2]);
        BR.setTargetPosition(targets[3]);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        thatYes = hardwareMap.get(Servo.class, "servo");
        thatYes.setPosition(.4);
        for (int i = 0; i < 4; i++) {
            targets[i] = 192;
        }
        telemetry.addData("Robot", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double speed = 0.3;
        boolean strafe = false;

        while (opModeIsActive()) {
            if (active) {
                if(!reset) {
                    FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    reset = true;
                }
                FL.setPower(speed);
                FR.setPower(speed);
                BL.setPower(speed);
                BR.setPower(speed);

                FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FL.setTargetPosition(targets[0]);
                FR.setTargetPosition(targets[1]);
                BL.setTargetPosition((int)(targets[2]/1.3872));
                BR.setTargetPosition(targets[3]);
            }
            if(gamepad1.dpad_right){
                speed = 0.7;
            }else if(gamepad1.dpad_left){
                speed = 0.3;
            }

            if((FR.isBusy()||FL.isBusy()||BL.isBusy()||BR.isBusy())) {


                FL.setPower(speed);
                FR.setPower(speed);
                BL.setPower(speed);
                BR.setPower(speed);
            }

            if((!FR.isBusy()&&!FL.isBusy()&&!BL.isBusy()&&!BR.isBusy())) {
                FL.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
                BR.setPower(0);
                active = false;
            }

            /**if(Math.abs(BR.getCurrentPosition()- BR.getTargetPosition())<=10){
                BL.setPower(0);
            }*/

            if (gamepad1.a&&!aPrev) {
                active = !active;

            }
            if(gamepad1.start){
                reset = false;
            }


            aPrev = gamepad1.a;


            if (gamepad1.x&&!xPrev) {
                x++;
                if (x > 3)
                    x = 0;
            }
            xPrev = gamepad1.x;
            if (gamepad1.dpad_down && !downD){
                threeTargets[3*x + modi]--;
                if(modi!=0){
                if(threeTargets[3*x + modi]<0){
                    threeTargets[3*x + modi] = 9;
                }}
            }else if(gamepad1.dpad_up && !upD){
                threeTargets[3*x + modi]++;
                if(modi!=0){
                    if(threeTargets[3*x + modi]>9){
                        threeTargets[3*x + modi] = 0;
                    }
                }
            }
            if(gamepad1.b&&!bPrev){
                modi ++;
                if(modi>2){
                    modi = 0;
                }
            }
            bPrev = gamepad1.b;
            downD = gamepad1.dpad_down;
            upD = gamepad1.dpad_up;

            resetTargets();
            telemetry.addData("Active", active);
            telemetry.addData("Wheel Power", "front left (%.2f), front right (%.2f), " +
                            "back left (%.2f), back right (%.2f)", FL.getPower(), FR.getPower(),
                    BL.getPower(), BR.getPower());
            telemetry.addData("X", x);
            telemetry.addData("Wheel Position", "front left (%.1f), front right (%.1f), " +
                            "back left (%.1f), back right (%.1f)", (float)FL.getCurrentPosition(), (float)FR.getCurrentPosition(),
                    (float)BL.getCurrentPosition(), (float)BR.getCurrentPosition());
            telemetry.addData("Target", "front left (%.1f), front right (%.1f), " +
                    "back left (%.1f), back right (%.1f)", (float)targets[0], (float)targets[1], (float)targets[2], (float)targets[3]);
            telemetry.addData("Real Target", "front left (%.1f), front right (%.1f), " +
                    "back left (%.1f), back right (%.1f)", (float)FL.getTargetPosition(), (float)FR.getTargetPosition(), (float)BL.getTargetPosition(), (float)BR.getTargetPosition());
            telemetry.addData("Modifier", modi);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
    private void resetTargets(){
        for(int i = 0; i<4; i++){
            targets[i] = threeTargets[3*i]*100 + threeTargets[3*i + 1] *10 + threeTargets[3*i + 2];
        }
    }
}

