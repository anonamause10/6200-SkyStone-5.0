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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.ftccommon.SoundPlayer;

import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorAdafruitRGB;
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
 * Created by isong on teh day afteh lague
 */
@TeleOp(group = "a", name="TelP LEDS")
@Disabled

public class distancetester extends LinearOpMode {
    // Declare OpMode members.
    PIDController           pidRotate;
    Orientation             lastAngles = new Orientation();
    private ElapsedTime runtim2 = new ElapsedTime();
    double globalAngle, rotation;
    private PIDController pidDrive;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx FL = null;
    private DcMotorEx FR = null;
    private DcMotorEx BL = null;
    private DcMotorEx BR = null;
    private DcMotor IN1 = null;
    private DcMotor IN2 = null;
    private DcMotor LIFT = null;
    private Servo rotateServo = null;
    private Servo clawServo = null;
    private Servo foundServL = null;
    private Servo foundServR = null;
    private DcMotor YEETER = null;
    boolean lightsaber = false;

    boolean clawClosed = false;

    private boolean dPadDPrev = false;
    private boolean dPadUPrev = false;
    private boolean dPadLPrev = false;
    private boolean dPadRPrev = false;
    private boolean sPrev = false;
    private DistanceSensor intSens = null;

    private boolean lbumpprev = false;
    private boolean rbumpprev = false;
    boolean backPrev = false;
    private DigitalChannel touch = null;
    // List of available sound resources
    String  sounds[] =  {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
            "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
            "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie" };
    boolean soundPlaying = false;
    RevBlinkinLedDriver blink;
    String pattern = "";
    String team = "red";
    // The IMU sensor object
    BNO055IMU imu;
    int ticks = 1000;
    double power = 0.5;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void runOpMode() throws InterruptedException {
        boolean yPrev     = false;
        boolean xPrev   = false;

        intSens = hardwareMap.get(DistanceSensor.class, "DS2");

        // create a sound parameter that holds the desired player parameters.
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        pidRotate = new PIDController(.003, .00003, 0);
        pidDrive = new PIDController(.05, 0, 0);
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, 1.0);
        pidDrive.setInputRange(-270, 270);
        pidDrive.enable();


        FL = hardwareMap.get(DcMotorEx.class, "fL");FR = hardwareMap.get(DcMotorEx.class, "fR");
        BL = hardwareMap.get(DcMotorEx.class, "bL");BR = hardwareMap.get(DcMotorEx.class, "bR");
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

        FL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(30.0, 1.0, 12.0, 12.0));
        FR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(30.0, 1.0, 12.0, 12.0));
        BL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(30.0, 1.0, 12.0, 12.0));
        BR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(30.0, 1.0, 12.0, 12.0));

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

        rotateServo= hardwareMap.get(Servo.class, "ROTATE");
        clawServo= hardwareMap.get(Servo.class, "CLAW");
        rotateServo.setPosition(0.69); clawServo.setPosition(0.15);

        foundServL = hardwareMap.get(Servo.class, "left");
        foundServR = hardwareMap.get(Servo.class, "right");
        foundServL.setPosition(0.2);
        foundServR.setPosition(.6);

        touch = hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);

        initIMU();
        blink = hardwareMap.get(RevBlinkinLedDriver.class, "blink");
        blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND);
        pattern = "endtoend";

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

            //DRIVE + MAYBE LED
            drive();
            if(gamepad1.left_bumper){
                IN1.setPower(-0.4);
                IN2.setPower(-0.4);
            }else if(gamepad1.right_bumper && intSens.getDistance(DistanceUnit.MM)>70){
                if(!pattern.equals("fire")){
                    blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_MEDIUM);
                    pattern = "fire";
                }
                IN1.setPower(0.7);
                IN2.setPower(0.7);
            }else{
                if(intSens.getDistance(DistanceUnit.MM)<70 && !pattern.equals("green")){
                    blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    pattern = "green";
                }
                IN1.setPower(0);
                IN2.setPower(0);
            }
            if(gamepad1.a && !pattern.equals("black")){
                blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                pattern = "black";
            }else if(gamepad1.b && !pattern.equals("endtoend")){
                blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND);
                pattern = "endtoend";
            }
            if(gamepad1.dpad_up){
                foundServL.setPosition(0);
                foundServR.setPosition(0.4);
            }else if(gamepad1.dpad_down){
                foundServL.setPosition(0.2);
                foundServR.setPosition(0.6);
            }

            if(gamepad2.start){
                blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                pattern = "blue";
                drive(ticks, power);
                blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                pattern = "red";
            }
            if(gamepad2.x&&!xPrev){
                ticks+=100;
            }
            if(gamepad2.y&&!yPrev){
                ticks-=100;
            }
            if(gamepad2.dpad_left&&!dPadLPrev){
                ticks-=10;
            }
            if(gamepad2.dpad_right&&!dPadRPrev){
                ticks+=10;
            }
            if(gamepad2.dpad_down&&!dPadDPrev){
                power-=0.05;
            }
            if(gamepad2.dpad_up&&!dPadUPrev){
                power+=0.05;
            }
            if(gamepad2.back&&!backPrev){
                ticks = -ticks;
            }

            //VARIABLE CHECKS

            xPrev = gamepad2.x;
            yPrev = gamepad2.y;
            dPadDPrev = gamepad2.dpad_down;
            dPadUPrev = gamepad2.dpad_up;
            dPadLPrev = gamepad2.dpad_left;
            dPadRPrev = gamepad2.dpad_right;
            backPrev = gamepad2.back;

            //TELEMETRY

            telemetry.addData("Wheel Power", "front left (%.2f), front right (%.2f), " +
                            "back left (%.2f), back right (%.2f)", FL.getPower(), FR.getPower(),
                    BL.getPower(), BR.getPower());
            telemetry.addData("power", power);
            telemetry.addData("ticks", ticks);
            telemetry.addData("LED Pattern", pattern);
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
        if(gamepad1.left_stick_button){
            v1 = v1 * .5;
            v2 = v2 * .5;
            v3 = v3 * .5;
            v4 = v4 * .5;
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

    public void drive(int ticks, double power){
        if(ticks <0){
            power = -power;
        }
        // restart imu angle tracking.
        resetAngle();
        int degrees = 0;

        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        pidDrive.reset();


        pidDrive.setSetpoint(0);
        pidDrive.setInputRange(0, degrees);
        pidDrive.setOutputRange(0, power);
        pidDrive.setTolerance(1);
        pidDrive.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.
        double pow = 0;
        runtim2.reset();
        boolean working = true;

        do
        {
            pow = pidDrive.performPID(getAngle()); // power will be + on left turn.
            updateT();
            FL.setPower(power-pow);
            FR.setPower(power+pow);
            BL.setPower(power-pow);
            BR.setPower(power+pow);
            if (Math.abs((4*ticks)-FL.getCurrentPosition()-FR.getCurrentPosition()-FL.getCurrentPosition()-BR.getCurrentPosition())<100){
                working = false;
            }
        } while (opModeIsActive() && runtim2.seconds()<1.5 && working );

        // turn the motors off.
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotation = getAngle();

        // wait for rotation to stop.

        // reset angle tracking on new heading.
        resetAngle();
    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    private void goV2(int ticks, double power){
        goV2(ticks, power, new double[]{0,0,0,0});
    }
    private void goV2(int ticks, double power, double[] endPowers){
        resetEncoders();
        runtim2.reset();
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
        FL.setPower(endPowers[0]);
        FR.setPower(endPowers[1]);
        BL.setPower(endPowers[2]);
        BR.setPower(endPowers[3]);
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    private void resetEncoders(){
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private double averageTicks(){
        return (FL.getCurrentPosition()+FR.getCurrentPosition()+BL.getCurrentPosition()+BR.getCurrentPosition())/4;
    }
    private void updateT(){
        telemetry.addData("Wheel Power", "front left (%.2f), front right (%.2f), " +
                        "back left (%.2f), back right (%.2f)", FL.getPower(), FR.getPower(),
                BL.getPower(), BR.getPower());
        telemetry.addData("Wheel Position", "front left (%.1f), front right (%.1f), " +
                        "back left (%.1f), back right (%.1f)", (float)FL.getCurrentPosition(), (float)FR.getCurrentPosition(),
                (float)BL.getCurrentPosition(), (float)BR.getCurrentPosition());
        telemetry.addData("Status", "Run Time2: " + runtim2.toString());
        telemetry.addData("INTAKE POWER", IN1.getPower());
        telemetry.update();
    }
}



