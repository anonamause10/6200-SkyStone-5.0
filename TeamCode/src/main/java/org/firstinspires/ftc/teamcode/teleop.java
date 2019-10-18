package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.ftccommon.SoundPlayer;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Created by isong on 10/17/18.
 */
@TeleOp(name="datOne")

public class teleop extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor IN1 = null;
    private DcMotor IN2 = null;
    //private DcMotor UD = null;
    private Servo S1 = null;
    private Servo S2 = null;
    private boolean xPrev = false;
    private boolean yPrev = false;
    private boolean sPrev = false;
    private double armpower = 0.7;
    private boolean lbumpprev = false;
    private boolean rbumpprev = false;
    // List of available sound resources
    String  sounds[] =  {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
            "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
            "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie" };
    boolean soundPlaying = false;





    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void runOpMode() throws InterruptedException {
        // Variables for choosing from the available sounds
        int     soundIndex      = 0;
        int     soundID         = -1;
        boolean was_dpad_up     = false;
        boolean was_dpad_down   = false;

        Context myApp = hardwareMap.appContext;

        // create a sound parameter that holds the desired player parameters.
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();

        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;
        FL = hardwareMap.get(DcMotor.class, "fL");
        FR = hardwareMap.get(DcMotor.class, "fR");
        BL = hardwareMap.get(DcMotor.class, "bL");
        BR = hardwareMap.get(DcMotor.class, "bR");
        FL.setDirection(REVERSE);
        BL.setDirection(REVERSE);
        FR.setDirection(FORWARD);
        BR.setDirection(FORWARD);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        IN1 = hardwareMap.get(DcMotor.class, "IN1");
        IN1.setDirection(FORWARD);
        IN2 = hardwareMap.get(DcMotor.class, "IN2");
        IN2.setDirection(FORWARD);

        /**UD = hardwareMap.get(DcMotor.class, "uD");
        UD.setDirection(DcMotor.Direction.FORWARD);
        UD.setPower(0);
        UD.setTargetPosition(0);
        UD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         */
        S1 = hardwareMap.get(Servo.class, "servo");
        S2 = hardwareMap.get(Servo.class, "servo2");
        S1.setPosition(1);
        S2.setPosition(1);

        telemetry.addData("Robot", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            drive();

            if (gamepad1.a) {
                S1.setPosition(0.5);
                S2.setPosition(0.6);
            } else if (gamepad1.b) {
                S1.setPosition(0.35);
                S2.setPosition(0.3);
            }else if(gamepad1.y){
                S1.setPosition(1);
                S2.setPosition(1);
            }
            double INSPEED = 0.4;
            if(gamepad1.left_bumper){
                IN1.setPower(-INSPEED);
                IN2.setPower(-INSPEED);
            }else if(gamepad1.right_bumper){
                IN1.setPower(0.7);
                IN2.setPower(0.7);
            }else{
                IN1.setPower(0);
                IN2.setPower(0);
            }
            if(gamepad1.left_trigger!=0){
                IN1.setPower(-gamepad1.left_trigger);
                IN2.setPower(-gamepad1.left_trigger);
            }else if(gamepad1.right_trigger!=0){
                IN1.setPower(gamepad1.right_trigger);
                IN2.setPower(gamepad1.right_trigger);
            }
            /**if (gamepad1.x && !xPrev) {
                UD.setTargetPosition(0);
                UD.setPower(0.7);
            } else if (gamepad1.y && !yPrev) {
                UD.setTargetPosition(60);
                UD.setPower(0.7);
            }
            if (gamepad1.start && !sPrev){
                UD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                UD.setPower(0);
                UD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                UD.setPower(0.7);
            }
            if(UD.getTargetPosition()==UD.getCurrentPosition()){
                UD.setPower(0);
            }*/

            xPrev = gamepad1.x;
            yPrev = gamepad1.y;
            rbumpprev = gamepad1.right_bumper;
            lbumpprev = gamepad1.left_bumper;


            telemetry.addData("Wheel Power", "front left (%.2f), front right (%.2f), " +
                            "back left (%.2f), back right (%.2f)", FL.getPower(), FR.getPower(),
                    BL.getPower(), BR.getPower());
            telemetry.addData("Wheel Position", "front left (%.1f), front right (%.1f), " +
                            "back left (%.1f), back right (%.1f)", (float)FL.getCurrentPosition(), (float)FR.getCurrentPosition(),
                    (float)BL.getCurrentPosition(), (float)BR.getCurrentPosition());
            //telemetry.addData("Set Arm Power:", armpower);
            /**telemetry.addData("Current Arm Position:", UD.getCurrentPosition());
            telemetry.addData("ArmPos", UD.getTargetPosition());
             */
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Servo1Pos", S1.getPosition());
            telemetry.addData("Servo2Pos", S2.getPosition());
            telemetry.addData("Sound >", sounds[soundIndex]);
            telemetry.addData("INTAKE POWER", IN1.getPower());
            telemetry.update();


            //lmao reee sound


            // Look for DPAD presses to change the selection
            if (gamepad2.dpad_down && !was_dpad_down) {
                // Go to next sound (with list wrap) and display it
                soundIndex = (soundIndex + 1) % sounds.length;
            }

            if (gamepad2.dpad_up && !was_dpad_up) {
                // Go to previous sound (with list wrap) and display it
                soundIndex = (soundIndex + sounds.length - 1) % sounds.length;
            }

            // Look for trigger to see if we should play sound
            // Only start a new sound if we are currently not playing one.
            if (gamepad2.right_bumper && !soundPlaying) {

                // Determine Resource IDs for the sounds you want to play, and make sure it's valid.
                if ((soundID = myApp.getResources().getIdentifier(sounds[soundIndex], "raw", myApp.getPackageName())) != 0){

                    // Signal that the sound is now playing.
                    soundPlaying = true;

                    // Start playing, and also Create a callback that will clear the playing flag when the sound is complete.
                    SoundPlayer.getInstance().startPlaying(myApp, soundID, params, null,
                            new Runnable() {
                                public void run() {
                                    soundPlaying = false;
                                }} );
                }
            }

            was_dpad_up     = gamepad2.dpad_up;
            was_dpad_down   = gamepad2.dpad_down;
        }
    }


    private void drive(){
        //DONT TOUCH THIS

        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = Math.pow(gamepad1.right_stick_x,5);
        //if(gamepad1.right_stick_x<0){
          //  rightX *= -1;
       // }
        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;
        if (gamepad1.x) {
            v1 *= 2.85;
            v2 *= 2.85;
            v3 *= 2.85;
            v4 *= 2.85;
        }
        FL.setPower(v1);
        FR.setPower(v2);
        BL.setPower(v3);
        BR.setPower(v4);
        //OK YOU GOOD NOW
    }
    private void sounds(int soundIndex, boolean was_dpad_down, boolean was_dpad_up, int soundID, Context myApp, SoundPlayer.PlaySoundParams params){


                // Look for DPAD presses to change the selection
                if (gamepad2.dpad_down && !was_dpad_down) {
                    // Go to next sound (with list wrap) and display it
                    soundIndex = (soundIndex + 1) % sounds.length;
                }

                if (gamepad2.dpad_up && !was_dpad_up) {
                    // Go to previous sound (with list wrap) and display it
                    soundIndex = (soundIndex + sounds.length - 1) % sounds.length;
                }

                // Look for trigger to see if we should play sound
                // Only start a new sound if we are currently not playing one.
                if (gamepad2.right_bumper && !soundPlaying) {

                    // Determine Resource IDs for the sounds you want to play, and make sure it's valid.
                    if ((soundID = myApp.getResources().getIdentifier(sounds[soundIndex], "raw", myApp.getPackageName())) != 0){

                        // Signal that the sound is now playing.
                        soundPlaying = true;

                        // Start playing, and also Create a callback that will clear the playing flag when the sound is complete.
                        SoundPlayer.getInstance().startPlaying(myApp, soundID, params, null,
                                new Runnable() {
                                    public void run() {
                                        soundPlaying = false;
                                    }} );
                    }
                }


    }
}


