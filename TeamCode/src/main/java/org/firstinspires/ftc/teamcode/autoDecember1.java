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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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
@TeleOp(group = "z", name="tester 2")

public class autoDecember1 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();
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
    private DcMotor YEETER = null;
    private DistanceSensor DSF = null;
    private double[] powers = {0.8, 0.05, 0.8, 0.05};

    private int indx = 0;
    private int indx2 = 0;

    private boolean dPadDPrev = false;
    private boolean dPadUPrev = false;
    private boolean dPadLPrev = false;
    private boolean dPadRPrev = false;
    private boolean drive = true;
    private boolean sPrev = false;
    private boolean aPrev = false;
    private double secondsToDisplay = 0;
    private double voltage = 0.0;
    private double scale = 0.0;


    double[] array1 = {0.68*scale, 0.2*scale, 0.68*scale, 0.2*scale, 850};
    double[] array0 = {0.1*scale, 0.88*scale, 0.1*scale, 0.88*scale, 850};
    double[] array2 = {0.9*scale, 0.1*scale, 0.9*scale, 0.1*scale, 380};
    double[][] arrays = new double[][]{array0, array1, array2};

    private boolean lbumpprev = false;
    private boolean rbumpprev = false;
    private boolean xprev = false;
    private boolean yprev = false;
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
        voltage = getBatteryVoltage();
        scale = 12.8 / voltage;




        int currLiftPos = 0;

        Context myApp = hardwareMap.appContext;
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;

        FL = hardwareMap.get(DcMotor.class, "FL");FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");BR = hardwareMap.get(DcMotor.class, "BR");
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

        YEETER = hardwareMap.get(DcMotor.class, "YEET");

        IN1 = hardwareMap.get(DcMotor.class, "IN1");
        IN1.setDirection(FORWARD);
        IN2 = hardwareMap.get(DcMotor.class, "IN2");
        IN2.setDirection(REVERSE);

        DSF = hardwareMap.get(DistanceSensor.class, "DSF");

        rotateServo= hardwareMap.get(Servo.class, "ROTATE");
        clawServo= hardwareMap.get(Servo.class, "CLAW");
        rotateServo.setPosition(0.69); clawServo.setPosition(0.15);

        foundServL = hardwareMap.get(Servo.class, "left");
        foundServR = hardwareMap.get(Servo.class, "right");
        foundServL.setPosition(0);
        foundServR.setPosition(.4);

        intSens = hardwareMap.get(DistanceSensor.class, "DS2");
        touch = hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);

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
            sPrev = gamepad2.back;

            if(gamepad2.dpad_up&&!dPadUPrev){
                indx++;
                if(indx>4)indx = 0;
            }else if(gamepad2.dpad_down&&!dPadDPrev){
                indx--;
                if(indx<0)indx = 4;
            }
            dPadDPrev = gamepad2.dpad_down;
            dPadUPrev = gamepad2.dpad_up;
            if(gamepad2.x&&!xprev){
                indx2++;
                if(indx2>2)indx2 = 0;
            }else if(gamepad2.y&&!yprev){
                indx2--;
                if(indx2<0)indx2 = 2;
            }
            xprev = gamepad2.x;
            yprev = gamepad2.y;

            if(gamepad2.dpad_left&&!dPadLPrev){
                if(indx == 4){
                    arrays[indx2][indx] = arrays[indx2][indx]-5;
                }else
                    arrays[indx2][indx] = arrays[indx2][indx]-0.05;
            }else if(gamepad2.dpad_right&&!dPadRPrev){
                if(indx == 4){
                    arrays[indx2][indx] = arrays[indx2][indx]+5;
                }else
                arrays[indx2][indx] = arrays[indx2][indx]+0.05;
            }
            dPadLPrev = gamepad2.dpad_left;
            dPadRPrev = gamepad2.dpad_right;

            if(gamepad1.dpad_up){
                foundServL.setPosition(0.4);
                foundServR.setPosition(0.65);
            }else if(gamepad1.dpad_down){
                foundServL.setPosition(0);
                foundServR.setPosition(0.4);
            }
            if(gamepad2.left_bumper&&!lbumpprev){
                if(foundServL.getPosition()==0){
                    foundServL.setPosition(0.4);
                    foundServR.setPosition(0.65);
                }else{
                    foundServL.setPosition(0);
                    foundServR.setPosition(0.4);
                }
            }
            lbumpprev = gamepad2.left_bumper;
            if(gamepad2.left_trigger>0){
                IN1.setPower(-0.4);
                IN2.setPower(-0.4);
            }
            if(gamepad2.start){
                resetEncoders();
            }

            if(gamepad2.a&&!drive){
                FL.setPower(arrays[indx2][0]);
                FR.setPower(arrays[indx2][1]);
                BL.setPower(arrays[indx2][2]);
                BR.setPower(arrays[indx2][3]);
                IN1.setPower(0.7);
                IN2.setPower(0.7);
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
            if(gamepad2.right_bumper){
                go((int)(arrays[indx2][4]), 0.5*scale);
                secondsToDisplay = runtime2.milliseconds();
            }
            aPrev = gamepad2.a;

            //TELEMETRY

            telemetry.addData("Wheel Position", "front left (%.1f), front right (%.1f), " +
                            "back left (%.1f), back right (%.1f)", FL.getCurrentPosition(), FR.getCurrentPosition(),
                    BL.getCurrentPosition(), BR.getCurrentPosition());
            telemetry.addData("Wheel Powers", "front left (%.2f), front right (%.2f), " +
                    "back left (%.2f), back right (%.2f)", arrays[indx2][0], arrays[indx2][1], arrays[indx2][2], arrays[indx2][3]);
            telemetry.addData("indx", indx);
            telemetry.addData("indx2", indx2);
            telemetry.addData("drive", drive);
            telemetry.addData("seconds2", secondsToDisplay);
            telemetry.addData("Distance", "millimeters " + DSF.getDistance(DistanceUnit.MM));
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
    private void updateT(){
        telemetry.addData("Wheel Position", "front left (%1f), front right (%1f), " +
                        "back left (%1f), back right (%1f)", FL.getCurrentPosition(), FR.getCurrentPosition(),
                BL.getCurrentPosition(), BR.getCurrentPosition());
        telemetry.addData("target pos", arrays[indx2][4]);
        telemetry.addData("indx", indx);
        telemetry.addData("indx2", indx2);
        telemetry.addData("drive", drive);
        telemetry.addData("runtim2", runtime2);
        telemetry.addData("Distance", "millimeters " + DSF.getDistance(DistanceUnit.MM));
        telemetry.update();
    }
    private void go(int ticks, double power){
        resetEncoders();
        runtime2.reset();
        if(ticks < 0){
            FL.setPower(-power);
            FR.setPower(-power);
            BL.setPower(-power);
            BR.setPower(-power);
            while(opModeIsActive()&&averageTicks()>ticks){
                updateT();
            }
        }else{
            FL.setPower(power);
            FR.setPower(power);
            BL.setPower(power);
            BR.setPower(power);
            while(opModeIsActive()&&averageTicks()<ticks){
                updateT();
            }
        }
        motorsOff();
    }
    private void resetEncoders(){
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void motorsOff(){
        FR.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
    private double averageTicks(){
        return (FR.getCurrentPosition()+FR.getCurrentPosition()+BL.getCurrentPosition()+BR.getCurrentPosition())/4;
    }
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}


