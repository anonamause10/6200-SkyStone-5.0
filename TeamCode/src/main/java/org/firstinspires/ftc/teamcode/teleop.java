package org.firstinspires.ftc.teamcode;

import android.content.Context;

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
@TeleOp(name="TeleOp")

public class teleop extends LinearOpMode {
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
    RevBlinkinLedDriver blinkinLedDriver;
    private String pattern = "";

    boolean clawClosed = false;

    private boolean dPadDPrev = false;
    private boolean dPadUPrev = false;
    private boolean dPadLPrev = false;
    private boolean dPadRPrev = false;
    private boolean xPrev = false;
    private boolean yPrev = false;
    private boolean sPrev = false;

    private double armpower = 0.7;
    private boolean lbumpprev = false;
    private boolean rbumpprev = false;
    private DistanceSensor intSens = null;
    private TouchSensor touchSens1 = null;
    // List of available sound resources
    String  sounds[] =  {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
            "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
            "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie" };
    boolean soundPlaying = false;
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;




    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void runOpMode() throws InterruptedException {
        // Variables for choosing from the available sounds
        int     soundIndex      = 0;
        int     soundID         = -1;
        boolean was_y_up     = false;
        boolean was_x_down   = false;

        double posIn = 0.69;
        double posOut = 0.025;
        double posClosed = 0.025;
        double posOpen = .15;
        int[] positions = {0, 300, 700, 1200, 1700};

        int currLiftPos = 0;

        Context myApp = hardwareMap.appContext;

        // create a sound parameter that holds the desired player parameters.
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blink");
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;
        FL = hardwareMap.get(DcMotor.class, "fL");
        FR = hardwareMap.get(DcMotor.class, "fR");
        BL = hardwareMap.get(DcMotor.class, "bL");
        BR = hardwareMap.get(DcMotor.class, "bR");
        FL.setDirection(REVERSE);BL.setDirection(REVERSE);
        FR.setDirection(FORWARD);BR.setDirection(FORWARD);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

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
        rotateServo.setPosition(posIn); clawServo.setPosition(posOpen);

        intSens = hardwareMap.get(DistanceSensor.class, "DS2");
        touchSens1 = hardwareMap.get(TouchSensor.class, "TOUCH1");

        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();

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
            drive(myApp, params);

            //CLAW STUFF

            if (gamepad2.a) {
                rotateServo.setPosition(posIn);
            }else if(gamepad2.b){
                rotateServo.setPosition(posOut);
            }

            if(gamepad2.right_bumper && !rbumpprev){

                if(!clawClosed) clawServo.setPosition(posClosed);//set to closed
                else clawServo.setPosition(posOpen); // set to open
                clawClosed = !clawClosed;
            }

            //ARM STUFF

            if(gamepad2.left_stick_y != 0.0 || gamepad2.right_stick_y != 0.0) {
                LIFT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (LIFT.getCurrentPosition() > 0)
                    LIFT.setPower(-gamepad2.left_stick_y + -gamepad2.right_stick_y*0.2);
                else if ((-gamepad2.left_stick_y) > 0)
                    LIFT.setPower(-gamepad2.left_stick_y);
                else
                    LIFT.setPower(0);
            }else{
                /**if(gamepad2.dpad_up) {
                    currLiftPos++;
                    LIFT.setTargetPosition(positions[currLiftPos]);
                }else if(gamepad2.dpad_down){
                    currLiftPos--;
                    LIFT.setTargetPosition(positions[currLiftPos]);
                }else if(gamepad2.dpad_left){
                    currLiftPos = 0;
                    LIFT.setTargetPosition(positions[currLiftPos]);
                }else if(gamepad2.dpad_right){
                    LIFT.setTargetPosition(positions[currLiftPos]);
                }else*/
                LIFT.setTargetPosition(LIFT.getCurrentPosition());
                LIFT.setPower(0.3);
                LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //INTAKE STUFF

            double INSPEED = -0.4;
            double OUTSPEED = 0.7;
            if(gamepad1.left_bumper && intSens.getDistance(DistanceUnit.MM)>70){
                IN1.setPower(INSPEED);
                IN2.setPower(INSPEED);
            }else if(gamepad1.right_bumper){
                IN1.setPower(OUTSPEED);
                IN2.setPower(OUTSPEED);
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

            //VARIABLE CHECKS

            xPrev = gamepad1.x;
            yPrev = gamepad1.y;

            rbumpprev = gamepad2.right_bumper;
            lbumpprev = gamepad2.left_bumper;
            dPadDPrev = gamepad2.dpad_down;
            dPadUPrev = gamepad2.dpad_up;
            dPadLPrev = gamepad2.dpad_left;
            dPadRPrev = gamepad2.dpad_right;


            telemetry.addData("Wheel Power", "front left (%.2f), front right (%.2f), " +
                            "back left (%.2f), back right (%.2f)", FL.getPower(), FR.getPower(),
                    BL.getPower(), BR.getPower());
            telemetry.addData("Lift:", "power (%.2f), real position (%.2f), currentLiftPos (%.2f)", LIFT.getPower(), LIFT.getCurrentPosition(), currLiftPos);
            telemetry.addData("Rotating Servo Pos", rotateServo.getPosition());
            telemetry.addData("Claw Poition", clawServo.getPosition());
            telemetry.addData("Sound >", sounds[soundIndex]);
            telemetry.addData("LED pattern", pattern);
            telemetry.addData("INTAKE POWER", IN1.getPower());
            telemetry.addData("IMU:", getHeading());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();


            //lmao reee sound


            // Look for DPAD presses to change the selection
            if (gamepad2.x && !was_x_down) {
                // Go to next sound (with list wrap) and display it
                soundIndex = (soundIndex + 1) % sounds.length;
            }

            if (gamepad2.y && !was_y_up) {
                // Go to previous sound (with list wrap) and display it
                soundIndex = (soundIndex + sounds.length - 1) % sounds.length;
            }

            // Look for trigger to see if we should play sound
            // Only start a new sound if we are currently not playing one.
            if (gamepad2.left_bumper && !soundPlaying) {

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

            was_y_up     = gamepad2.y;
            was_x_down   = gamepad2.x;
        }
    }


    private void drive(Context myApp, SoundPlayer.PlaySoundParams params){
        //DONT TOUCH THIS

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
        if (gamepad1.left_stick_button||gamepad1.right_stick_button) {
            v1 *= 2.85;
            v2 *= 2.85;
            v3 *= 2.85;
            v4 *= 2.85;
        }

        if(intSens.getDistance(DistanceUnit.MM)<70) {
            pattern = "CONFETTI";
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
        }else if(v1 < 0 && v2 < 0 && v3 < 0 && v4 <0){
            pattern = "STROBE_RED";
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);

            /**int soundID  = -1;
            if(!soundPlaying){
                // Determine Resource IDs for the sounds you want to play, and make sure it's valid.
                if ((soundID = myApp.getResources().getIdentifier(sounds[0], "raw", myApp.getPackageName())) != 0){

                    // Signal that the sound is now playing.
                    soundPlaying = true;

                    // Start playing, and also Create a callback that will clear the playing flag when the sound is complete.
                    SoundPlayer.getInstance().startPlaying(myApp, soundID, params, null,
                            new Runnable() {
                                public void run() {
                                    soundPlaying = false;
                                }} );
                }
            }*/
        }else{
            pattern = "CP1_2_SINELON";
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SINELON);
        }
        FL.setPower(v1);
        FR.setPower(v2);
        BL.setPower(v3);
        BR.setPower(v4);
        //OK YOU GOOD NOW
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle+360)%360;
    }
}


