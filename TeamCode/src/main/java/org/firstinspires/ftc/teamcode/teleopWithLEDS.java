package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.text.method.Touch;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.ftccommon.SoundPlayer;

import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Created by isong on 10/17/18.
 */
@Disabled
@TeleOp(group = "z", name="TeleOp with LEDS")

public class teleopWithLEDS extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor IN1 = null;
    private DcMotor IN2 = null;
    private DcMotor LIFT = null;
    private Servo rotateServo = null;
    private Servo clawServo = null;
    private Servo foundServL = null;
    private Servo foundServR = null;
    RevBlinkinLedDriver blinkinLedDriver;
    private String pattern = "";

    boolean clawClosed = false;

    private boolean dPadDPrev = false;
    private boolean dPadUPrev = false;
    private boolean dPadLPrev = false;
    private boolean dPadRPrev = false;
    private boolean sPrev = false;


    private boolean lbumpprev = false;
    private boolean rbumpprev = false;
    private DistanceSensor intSens = null;
    private TouchSensor touch = null;
    // List of available sound resources
    String  sounds[] =  {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
            "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
            "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie" };
    boolean soundPlaying = false;
    // The IMU sensor object
    BNO055IMU imu;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void runOpMode() throws InterruptedException {
        // Variables for choosing from the available sounds
        int     soundIndex      = 0;
        int     soundID         = -1;
        boolean yPrev     = false;
        boolean xPrev   = false;
        boolean liftRunning = false;


        int[] positions = {0, 111, 391, 599, 879, 1099, 1299, 1499, 1699};

        int currLiftPos = 0;

        Context myApp = hardwareMap.appContext;
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blink");

        FL = hardwareMap.get(DcMotor.class, "fL");FR = hardwareMap.get(DcMotor.class, "fR");
        BL = hardwareMap.get(DcMotor.class, "bL");BR = hardwareMap.get(DcMotor.class, "bR");
        FL.setDirection(REVERSE);FR.setDirection(FORWARD);
        BL.setDirection(REVERSE);BR.setDirection(FORWARD);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setPower(0);FR.setPower(0);
        BL.setPower(0);BR.setPower(0);

        LIFT = hardwareMap.get(DcMotor.class, "LIFT");
        LIFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LIFT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LIFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LIFT.setPower(0);
        LIFT.setTargetPosition(LIFT.getCurrentPosition());

        IN1 = hardwareMap.get(DcMotor.class, "IN1");
        IN1.setDirection(FORWARD);
        IN2 = hardwareMap.get(DcMotor.class, "IN2");
        IN2.setDirection(REVERSE);

        rotateServo= hardwareMap.get(Servo.class, "ROTATE");
        clawServo= hardwareMap.get(Servo.class, "CLAW");
        rotateServo.setPosition(0.69); clawServo.setPosition(0.15);

        foundServL = hardwareMap.get(Servo.class, "left");
        foundServR = hardwareMap.get(Servo.class, "right");
        foundServL.setPosition(.7);
        foundServR.setPosition(.3);

        intSens = hardwareMap.get(DistanceSensor.class, "DS2");
        touch = hardwareMap.get(TouchSensor.class, "touch");

        //initIMU();
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND);
        pattern = "CP1_2_END_TO_END_BLEND";
        telemetry.addData("Robot", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        runtime.reset();
        while (opModeIsActive()) {

            //DRIVE + MAYBE LED + PROBABLY NOT SOUND STUFF
            drive();

            //CLAW STUFF
            if (gamepad2.a) {
                rotateServo.setPosition(0.69);
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND);
                pattern = "CP1_2_END_TO_END_BLEND";
            }else if(gamepad2.b){
                rotateServo.setPosition(0.025);

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

            if(gamepad2.left_bumper) {
                clawServo.setPosition(0.15);
            }else if(gamepad2.right_bumper){
                clawServo.setPosition(0);
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);
                pattern = "CP1_2_COLOR_GRADIENT";
            }else if (intSens.getDistance(DistanceUnit.MM)<70 && LIFT.getCurrentPosition()<30 && clawServo.getPosition()!=0){
                clawServo.setPosition(0);
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                pattern = "GREEN";
            }

            //ARM STUFF

            if(gamepad2.start && gamepad2.back){
                LIFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LIFT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if(gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0) {
                if(touch.isPressed()){
                    LIFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    LIFT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    LIFT.setPower(0);
                }else if (-gamepad2.left_stick_y>=0){
                    if(!pattern.equals("CP1_2_SINELON")) {
                        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SINELON);
                        pattern = "CP1_2_SINELON";
                    }
                    LIFT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    LIFT.setPower(-gamepad2.left_stick_y + -gamepad2.right_stick_y * 0.25);
                }else{
                    LIFT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    LIFT.setPower(-0.5 * gamepad2.left_stick_y + -gamepad2.right_stick_y * 0.25 + 0.2);
                }
            }else{

                if(LIFT.getCurrentPosition()>50){
                    LIFT.setTargetPosition(LIFT.getCurrentPosition()); //STALL
                    LIFT.setPower(0.2);

                    if(pattern.equals("CP1_2_SINELON")) {
                        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_SLOW);
                        pattern = "CP2_BREATH_SLOW";
                    }

                }else if(LIFT.getCurrentPosition()<=5 && LIFT.getMode()==DcMotor.RunMode.RUN_TO_POSITION){
                    LIFT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }else{
                    LIFT.setPower(0);
                }
            }
            dPadDPrev = gamepad2.dpad_down; dPadUPrev = gamepad2.dpad_up;
            dPadLPrev = gamepad2.dpad_left; dPadRPrev = gamepad2.dpad_right;

            //INTAKE STUFF

            if(gamepad1.left_bumper){
                IN1.setPower(-0.4);
                IN2.setPower(-0.4);
            }else if(gamepad1.right_bumper && intSens.getDistance(DistanceUnit.MM)>70){
                IN1.setPower(0.7);
                IN2.setPower(0.7);
                if(!pattern.equals("CP1_SHOT")) {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT);
                    pattern = "CP1_SHOT";
                }
            }else{
                IN1.setPower(0);
                IN2.setPower(0);
            }

            /**if(gamepad1.left_trigger != 0){
             IN1.setPower(-gamepad1.left_trigger);
             IN2.setPower(-gamepad1.left_trigger);
             }else if(gamepad1.right_trigger != 0){
             IN1.setPower(gamepad1.right_trigger);
             IN2.setPower(gamepad1.right_trigger);
             }*/

            //FOUNDATION

            if(gamepad1.dpad_up){
                foundServL.setPosition(0.5);
                foundServR.setPosition(0.5);
            }else if(gamepad1.dpad_down){
                foundServL.setPosition(0.7);
                foundServR.setPosition(0.3);
            }

            //VARIABLE CHECKS

            xPrev = gamepad2.x;
            yPrev = gamepad2.y;

            //TELEMETRY

            telemetry.addData("Wheel Power", "front left (%.2f), front right (%.2f), " +
                            "back left (%.2f), back right (%.2f)", FL.getPower(), FR.getPower(),
                    BL.getPower(), BR.getPower());
            telemetry.addData("Lift:", LIFT.getPower());
            telemetry.addData("Lif2:",LIFT.getCurrentPosition());
            telemetry.addData("DISTANCE", intSens.getDistance(DistanceUnit.MM));
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

        }
    }


    private void drive(){

        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = Math.pow(gamepad1.right_stick_x,4);
        if(gamepad1.right_stick_x<0){
            rightX *= -1;
        }
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
        if(gamepad1.left_stick_button || gamepad1.right_stick_button || gamepad1.left_trigger!=0 || gamepad1.right_trigger!=0){
            v1 *= .5;
            v2 *= .5;
            v3 *= .5;
            v4 *= .5;
        }
        FL.setPower(v1);
        FR.setPower(v2);
        BL.setPower(v3);
        BR.setPower(v4);
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle+360)%360;
    }
    private void initIMU(){
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
}

