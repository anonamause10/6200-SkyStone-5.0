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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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

/**
 * Created by isong on 10/17/18.
 */
@TeleOp(group = "z", name="spline tester")

public class splinexD extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();
    private DcMotorEx FL = null;
    private DcMotorEx FR = null;
    private DcMotorEx BL = null;
    private DcMotorEx BR = null;
    private DcMotorEx IN1 = null;
    private DcMotorEx IN2 = null;
    private DcMotorEx LIFT = null;
    private Servo rotateServo = null;
    private Servo clawServo = null;
    private Servo foundationServo = null;
    private DcMotorEx YEETER = null;
    private double[] powers = {800, 200, 800, 200};
    boolean blockPushed = false;
    double starttime = 0;

    private Servo blockPusher = null;

    private int indx = 0;

    private boolean dPadDPrev = false;
    private boolean dPadUPrev = false;
    private boolean dPadLPrev = false;
    private boolean dPadRPrev = false;
    private boolean drive = true;
    private boolean sPrev = false;
    private boolean aPrev = false;
    private double secondsToDisplay = 0;
    private boolean intakeDuringGo = true;

    private boolean lbumpprev = false;
    private boolean rbumpprev = false;
    private DistanceSensor intSens = null;
    private DigitalChannel touch = null;
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
        int     soundIndex      = 8;
        boolean yPrev     = false;
        boolean xPrev   = false;
        boolean liftRunning = false;


        int[] positions = {0, 111, 391, 599, 879, 1099, 1299, 1499, 1699};

        int currLiftPos = 0;

        Context myApp = hardwareMap.appContext;
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;

        FL = hardwareMap.get(DcMotorEx.class, "fL");FR = hardwareMap.get(DcMotorEx.class, "fR");
        BL = hardwareMap.get(DcMotorEx.class, "bL");BR = hardwareMap.get(DcMotorEx.class, "bR");
        FL.setDirection(DcMotorEx.Direction.REVERSE);FR.setDirection(DcMotorEx.Direction.FORWARD);
        BL.setDirection(DcMotorEx.Direction.REVERSE);BR.setDirection(DcMotorEx.Direction.FORWARD);
        FL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);FR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);BR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);FR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);BR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FL.setPower(0);FR.setPower(0);
        BL.setPower(0);BR.setPower(0);

        LIFT = hardwareMap.get(DcMotorEx.class, "LIFT");
        LIFT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LIFT.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        LIFT.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LIFT.setPower(0);
        LIFT.setTargetPosition(LIFT.getCurrentPosition());

        YEETER = hardwareMap.get(DcMotorEx.class, "YEET");

        IN1 = hardwareMap.get(DcMotorEx.class, "IN1");
        IN1.setDirection(DcMotor.Direction.FORWARD);
        IN2 = hardwareMap.get(DcMotorEx.class, "IN2");
        IN2.setDirection(DcMotor.Direction.REVERSE);

        rotateServo= hardwareMap.get(Servo.class, "ROTATE");
        clawServo= hardwareMap.get(Servo.class, "CLAW");
        rotateServo.setPosition(0.84); clawServo.setPosition(0.2);

        foundationServo = hardwareMap.get(Servo.class, "left");

        intSens = hardwareMap.get(DistanceSensor.class, "DS2");
        touch = hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);
        blockPusher = hardwareMap.get(Servo.class, "push");
        blockPusher.setPosition(0);

        //initIMU();

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

            if(drive)
                drive();
            if(gamepad2.back && !sPrev){
                drive = !drive;
            }
            if(gamepad2.back && !sPrev && gamepad2.start){
                intakeDuringGo = !intakeDuringGo;
            }
            sPrev = gamepad2.back;

            if(gamepad2.dpad_up&&!dPadUPrev){
                indx++;
                if(indx>3)indx = 0;
            }else if(gamepad2.dpad_down&&!dPadDPrev){
                indx--;
                if(indx<0)indx = 3;
            }
            dPadDPrev = gamepad2.dpad_down;
            dPadUPrev = gamepad2.dpad_up;

            if(gamepad2.dpad_left&&!dPadLPrev){
                powers[indx] = powers[indx]-10;
            }else if(gamepad2.dpad_right&&!dPadRPrev){
                powers[indx] = powers[indx]+10;
            }
            if(gamepad2.left_bumper) {
                clawServo.setPosition(0.2);
            }
            dPadLPrev = gamepad2.dpad_left;
            dPadRPrev = gamepad2.dpad_right;

            if(gamepad1.dpad_up){
                foundationServo.setPosition(0.87);
            }else if(gamepad1.dpad_down){
                foundationServo.setPosition(0.4);
            }else if(gamepad1.dpad_right)
                foundationServo.setPosition(0.55);
            if(gamepad2.a){
                FL.setVelocity(powers[0]);
                FR.setVelocity(powers[1]);
                BL.setVelocity(powers[2]);
                BR.setVelocity(powers[3]);
                if(intakeDuringGo) {
                    IN1.setPower(0.7);
                    IN2.setPower(0.7);
                }
                if(!aPrev)
                    runtime2.reset();
            }else if(!drive){
                FL.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
                BR.setPower(0);
                IN1.setPower(0);
                IN2.setPower(0);
                if(aPrev){
                    secondsToDisplay = runtime2.milliseconds();
                }
            }
            aPrev = gamepad2.a;

            //TELEMETRY

            telemetry.addData("Wheel Power", "front left (%.2f), front right (%.2f), " +
                            "back left (%.2f), back right (%.2f)", FL.getPower(), FR.getPower(),
                    BL.getPower(), BR.getPower());

            telemetry.addData("Wheel Powers", "front left (%.2f), front right (%.2f), " +
            "back left (%.2f), back right (%.2f)", powers[0], powers[1], powers[2], powers[3]);
            telemetry.addData("indx", indx);
            telemetry.addData("drive", drive);
            telemetry.addData("seconds2", secondsToDisplay);
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
    private void playSound(int soundIndex, Context myApp, SoundPlayer.PlaySoundParams params){
        // Determine Resource IDs for the sounds you want to play, and make sure it's valid.
        int soundID = -1;
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


