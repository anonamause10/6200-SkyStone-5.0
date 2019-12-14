/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Autonomous(name="BLUE STONE", group="ree")

public class bSauto extends LinearOpMode
{

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtim2 = new ElapsedTime();

    private DcMotor fL = null;
    private DcMotor fR = null;
    private DcMotor bL = null;
    private DcMotor bR = null;
    private DcMotor IN1 = null;
    private DcMotor IN2 = null;
    private DcMotor LIFT = null;
    private Servo CLAW =null;
    private Servo ROTATE = null;
    private Servo servo = null;
    private Servo servo2 = null;
    private DistanceSensor sR;
    private DistanceSensor sR2;
    private DistanceSensor sRF;
    private DistanceSensor sRL;
    private TouchSensor touch;

    private double voltage = 0.0;
    private double scale = 0.0;
    private int blockPos = 1;
    PIDController           pidRotate;
    Orientation             lastAngles = new Orientation();
    double globalAngle, rotation;


    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    String  sounds[] =  {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
            "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
            "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie" };
    boolean soundPlaying = false;


    //for right with camera: pos 0 is 0, pos 1 is 114, pos 2 is 284

    @Override
    public void runOpMode() {
        // Variables for choosing from the available sounds
        int     soundIndex      = 0;
        int     soundID         = -1;
        boolean was_dpad_up     = false;
        boolean was_dpad_down   = false;

        Context myApp = hardwareMap.appContext;

        // create a sound parameter that holds the desired player parameters.
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        pidRotate = new PIDController(.003, .00003, 0);

        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;
        fL= hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        fL.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.FORWARD);
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LIFT = hardwareMap.get(DcMotor.class, "LIFT");
        LIFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IN1 = hardwareMap.get(DcMotor.class, "IN1");
        IN1.setDirection(DcMotor.Direction.FORWARD);
        IN2 = hardwareMap.get(DcMotor.class,"IN2");
        IN2.setDirection(DcMotor.Direction.REVERSE);
        CLAW = hardwareMap.get(Servo.class, "CLAW");
        CLAW.setPosition(.25);
        ROTATE = hardwareMap.get(Servo.class, "ROTATE");
        ROTATE.setPosition(.67);
        servo = hardwareMap.get(Servo.class, "left");
        servo2 = hardwareMap.get(Servo.class, "right");
        servo.setPosition(.7);
        servo2.setPosition(.3);

        SkystoneDetector sky = new SkystoneDetector(hardwareMap, true, true, false);

        sR = hardwareMap.get(DistanceSensor.class, "DSB");
        sR2 = hardwareMap.get(DistanceSensor.class, "DS2");
        sRL = hardwareMap.get(DistanceSensor.class, "DSL");
        sRF = hardwareMap.get(DistanceSensor.class, "DSF");
        touch = hardwareMap.get(TouchSensor.class, "touch");


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
        String angle = formatAngle(angles.angleUnit, angles.firstAngle);
        double ang = Double.parseDouble(angle);

        telemetry.addData("Robot", "Initialized");
        voltage = getBatteryVoltage();
        scale = 12.7 / voltage;
        telemetry.addData("Voltage:", voltage);
        telemetry.addData("Scale", scale);
        telemetry.update();
        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.addData("status", "waiting for start command...");
            telemetry.addData("Stone curr dist: ", sky.getDist());
            telemetry.addData("Stone Current pos", rundetect(sky));
            telemetry.update();
        }
        runtime.reset();

        LIFT.setPower(0.7);
        sleep(400);
        LIFT.setTargetPosition(0);
        LIFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LIFT.setPower(-0.5);
        runtim2.reset();
        while(opModeIsActive()&&(!touch.isPressed() || LIFT.getCurrentPosition() < -5) && runtim2.seconds()<0.7){
        }
        LIFT.setPower(0);
        LIFT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ROTATE.setPosition(0.69);
        CLAW.setPosition(0.15);


        //START AUTO HERE LMAO
        blockPos = rundetect(sky);
        sky.stop();
        moveWithForwardSensor(210, 0.35);




        goUntilBlock(0.3);
        if(blockPos == 1){
            strafe(450, 0.7);
        }

        intakeOff();
        turn(92);

        if(sRL.getDistance(DistanceUnit.MM)<635) {
            moveWithLeftSensor(655, 0.35);
        }else if (sRL.getDistance(DistanceUnit.MM)>670)
            moveWithLeftSensor(655, 0.4);{
        }

        go(2100, 0.6);
        turn(92);
        outtake();

        if(sRL.getDistance(DistanceUnit.MM)<635) {
            moveWithLeftSensor(655, 0.35);
        }else if (sRL.getDistance(DistanceUnit.MM)>670)
            moveWithLeftSensor(655, 0.4);{
        }

        go(-2200, 0.6);

        turn(90);
        intakeOff();

        if(sRL.getDistance(DistanceUnit.MM)<630) {
            moveWithLeftSensor(655, 0.3);
        }else if (sRL.getDistance(DistanceUnit.MM)>670)
            moveWithLeftSensor(655, 0.3);{
        }

        if(blockPos == 0)
            moveWithBackSensor(650, 0.4);
        else if(blockPos == 1)
            moveWithBackSensor(580, 0.4);
        else
            moveWithBackSensor(400, 0.4);

        turn(320);

        goUntilBlock(0.4);

        turn(90);
        moveWithBackSensor(645, 0.5);


        if(sRL.getDistance(DistanceUnit.MM)<640) {
            moveWithLeftSensor(655, 0.35);
        }else if (sRL.getDistance(DistanceUnit.MM)>670)
            moveWithLeftSensor(655, 0.4);{
        }

        turn(92);
        go(1900,0.6);
        turn(92);

        if(sRL.getDistance(DistanceUnit.MM)<640) {
            moveWithLeftSensor(655, 0.5);
        }else if (sRL.getDistance(DistanceUnit.MM)>670)
            moveWithLeftSensor(655, 0.5);{
        }

        outtake();
        go(-780,0.8);
        intakeOff();

    }

    private void go(int ticks, double power){
        fL.setTargetPosition(ticks);
        fR.setTargetPosition(ticks);
        bL.setTargetPosition(ticks);
        bR.setTargetPosition(ticks);
        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);

        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtim2.reset();
        boolean working = true;
        while(opModeIsActive() && fL.isBusy()&& fR.isBusy() && bL.isBusy() && bR.isBusy() && runtim2.seconds()<3 && working) {
            updateT();
            if (Math.abs(fL.getCurrentPosition() - fL.getTargetPosition())
                    + Math.abs(fR.getCurrentPosition() - fR.getTargetPosition())
                    + Math.abs(bL.getCurrentPosition() - bL.getTargetPosition())
                    + Math.abs(bR.getCurrentPosition() - bR.getTargetPosition())
                    < 60) {
                working = false;
            }
        }
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void goUntilBlock(double power){
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake();
        sleep(200);
        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtim2.reset();
        while(opModeIsActive()&& runtim2.seconds()<2.5 && sR2.getDistance(DistanceUnit.MM)>70) {
            updateT();
        }
        intakeOff();
        fL.setTargetPosition(0);
        fR.setTargetPosition(0);
        bL.setTargetPosition(0);
        bR.setTargetPosition(0);
        fL.setPower(-0.5);
        fR.setPower(-0.5);
        bL.setPower(-0.5);
        bR.setPower(-0.5);
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean working = true;
        while(opModeIsActive() && fL.isBusy()&& fR.isBusy() && bL.isBusy() && bR.isBusy() && runtim2.seconds()<=2.5 && working) {
            updateT();
            if (Math.abs(fL.getCurrentPosition() - fL.getTargetPosition())
                    + Math.abs(fR.getCurrentPosition() - fR.getTargetPosition())
                    + Math.abs(bL.getCurrentPosition() - bL.getTargetPosition())
                    + Math.abs(bR.getCurrentPosition() - bR.getTargetPosition())
                    < 30) {
                working = false;
            }
        }
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void strafe(int ticks, double power){
        fL.setTargetPosition(ticks);
        fR.setTargetPosition(-ticks);
        bL.setTargetPosition(-ticks);
        bR.setTargetPosition(ticks);
        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);

        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtim2.reset();
        boolean working = true;
        while(opModeIsActive() && fL.isBusy()&& fR.isBusy() && bL.isBusy() && bR.isBusy() && runtim2.seconds()<1 && working) {
            updateT();
            if (Math.abs(fL.getCurrentPosition() - fL.getTargetPosition())
                    + Math.abs(fR.getCurrentPosition() - fR.getTargetPosition())
                    + Math.abs(bL.getCurrentPosition() - bL.getTargetPosition())
                    + Math.abs(bR.getCurrentPosition() - bR.getTargetPosition())
                    < 80) {
                working = false;
            }
        }
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle+360)%360;
    }

    private void rotateIn(){
        ROTATE.setPosition(0.69);
        sleep(100);
    }
    private void rotateOut(){
        ROTATE.setPosition(0.025);
        sleep(100);
    }
    private void closeClaw(){
        CLAW.setPosition(0);
        sleep(100);
    }
    private void openClaw(){
        CLAW.setPosition(0.15);
        sleep(100);
    }
    private void servosUp(){
        servo.setPosition(.5);
        servo2.setPosition(.5);
        sleep(100);
    }
    private void servosDown(){
        servo.setPosition(.7);
        servo2.setPosition(.3);
        sleep(100);
    }

    private void intake(){
        IN1.setPower(0.7);
        IN2.setPower(0.7);
    }
    private void outtake(){
        IN1.setPower(-0.6);
        IN2.setPower(-0.6);
    }
    private void intakeOff(){
        IN1.setPower(0);
        IN2.setPower(0);
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

    private void updateT(){
        telemetry.addData("Wheel Power", "front left (%.2f), front right (%.2f), " +
                        "back left (%.2f), back right (%.2f)", fL.getPower(), fR.getPower(),
                bL.getPower(), bR.getPower());
        telemetry.addData("Wheel Position", "front left (%.1f), front right (%.1f), " +
                        "back left (%.1f), back right (%.1f)", (float)fL.getCurrentPosition(), (float)fR.getCurrentPosition(),
                (float)bL.getCurrentPosition(), (float)bR.getCurrentPosition());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status", "Run Time2: " + runtim2.toString());
        telemetry.addData("BackSensor", sR.getDistance(DistanceUnit.CM));
        telemetry.addData("BlockPos", blockPos);
        telemetry.addData("INTAKE POWER", IN1.getPower());
        telemetry.update();
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    void moveWithBackSensor(int target, double power){

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtim2.reset();
        if(sR.getDistance(DistanceUnit.MM)>target){
            while(opModeIsActive()&&(sR.getDistance(DistanceUnit.MM) > target)&&runtim2.seconds()<5){
                fL.setPower(-power);
                fR.setPower(-power);
                bL.setPower(-power);
                bR.setPower(-power);
                updateT();
            }}else{
            while(opModeIsActive()&&(sR.getDistance(DistanceUnit.MM) < target)&&runtim2.seconds()<5){
                fL.setPower(power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(power);
                updateT();
            }
        }
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    void moveWithForwardSensor(int target, double power){

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtim2.reset();
        if(sRF.getDistance(DistanceUnit.MM)>target){
            while(opModeIsActive()&&(sRF.getDistance(DistanceUnit.MM) > target)&&runtim2.seconds()<5){
                fL.setPower(power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(power);
                updateT();
            }}else{
            while(opModeIsActive()&&(sRF.getDistance(DistanceUnit.MM) > target)&&runtim2.seconds()<5){
                fL.setPower(-power);
                fR.setPower(-power);
                bL.setPower(-power);
                bR.setPower(-power);
                updateT();
            }
        }
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    void moveWithLeftSensor(int target, double power){

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtim2.reset();
        double startAngle = getHeading();
        if(sRL.getDistance(DistanceUnit.MM)<target){
            while(opModeIsActive()&&(sRL.getDistance(DistanceUnit.MM) < target)&&runtim2.seconds()<1){
                fL.setPower(power);
                fR.setPower(-power);
                bL.setPower(-power);
                bR.setPower(power);

                if(startAngle==0) {
                    if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle>0.25){
                        fL.setPower(fL.getPower()-0.02);
                        fR.setPower(fR.getPower()+0.02);
                        bL.setPower(bL.getPower()-0.02);
                        bR.setPower(bR.getPower()+0.02);
                    }else if((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle<0.25)){
                        fL.setPower(fL.getPower()+0.02);
                        fR.setPower(fR.getPower()-0.02);
                        bL.setPower(bL.getPower()+0.02);
                        bR.setPower(bR.getPower()-0.02);
                    }else{}
                }else {
                    if (getHeading() - startAngle > 0.25) {
                        fL.setPower(fL.getPower() - 0.02);
                        fR.setPower(fR.getPower() + 0.02);
                        bL.setPower(bL.getPower() - 0.02);
                        bR.setPower(bR.getPower() + 0.02);
                    } else if ((getHeading() - startAngle < 0.25)) {
                        fL.setPower(fL.getPower() + 0.02);
                        fR.setPower(fR.getPower() - 0.02);
                        bL.setPower(bL.getPower() + 0.02);
                        bR.setPower(bR.getPower() - 0.02);
                    }else{}
                }
                updateT();
            }}else{
            while(opModeIsActive()&&(sRL.getDistance(DistanceUnit.MM) > target)&&runtim2.seconds()<1){
                fL.setPower(-power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(-power);
                if(startAngle==0) {
                    if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle<0.25){
                        fL.setPower(fL.getPower()-0.02);
                        fR.setPower(fR.getPower()+0.02);
                        bL.setPower(bL.getPower()-0.02);
                        bR.setPower(bR.getPower()+0.02);
                    }else if((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle>0.25)){
                        fL.setPower(fL.getPower()+0.02);
                        fR.setPower(fR.getPower()-0.02);
                        bL.setPower(bL.getPower()+0.02);
                        bR.setPower(bR.getPower()-0.02);
                    }else{}
                }else {
                    if (getHeading() - startAngle < 0.25) {
                        fL.setPower(fL.getPower() - 0.02);
                        fR.setPower(fR.getPower() + 0.02);
                        bL.setPower(bL.getPower() - 0.02);
                        bR.setPower(bR.getPower() + 0.02);
                    } else if ((getHeading() - startAngle > 0.25)) {
                        fL.setPower(fL.getPower() + 0.02);
                        fR.setPower(fR.getPower() - 0.02);
                        bL.setPower(bL.getPower() + 0.02);
                        bR.setPower(bR.getPower() - 0.02);
                    }else{}
                }
                updateT();

            }
        }
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
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


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {

        double currheading = getHeading();
        double deltaang = Math.abs(degrees- currheading);
        double r = (deltaang >180? (360-deltaang):deltaang);
        int sign = (degrees - currheading >= 0 && degrees - currheading <= 180) || (degrees - currheading <=-180 && degrees- currheading>= -360) ? 1 : -1;
        degrees = Math.round((long)(r*sign));

        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);


        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();

        double p = Math.abs(power/degrees);
        double i = p / 100.0;
        pidRotate.setPID(p, i, 0);

        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                fL.setPower(power);
                fR.setPower(-power);
                bL.setPower(power);
                bR.setPower(-power);
                //sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                fL.setPower(-power);
                fR.setPower(power);
                bL.setPower(-power);
                bR.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                fL.setPower(-power);
                fR.setPower(power);
                bL.setPower(-power);
                bR.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }


    void turn(double tun){
        double vuAng = tun;
        boolean turned = false;
        while (!turned && opModeIsActive()) {
            double ang = getHeading();

            telemetry.addData("Angle", ang);
            telemetry.addData("TurnTo", vuAng);
            telemetry.addData("Wheel Power", "front left (%.2f), front right (%.2f), " +
                            "back left (%.2f), back right (%.2f)", fL.getPower(), fR.getPower(),
                    bL.getPower(), bR.getPower());
            telemetry.update();
            if(Math.abs(ang - vuAng) <= 0.5){
                fL.setPower(0);
                fR.setPower(0);
                bL.setPower(0);
                bR.setPower(0);
            }else if(ang>=260&& vuAng<=100){
                fL.setPower(-0.3);
                fR.setPower(0.3);
                bL.setPower(-0.3);
                bR.setPower(0.3);
            }else if(ang<=100&& vuAng>=260){
                fL.setPower(0.3);
                fR.setPower(-0.3);
                bL.setPower(0.3);
                bR.setPower(-0.3);
            }else if (ang-vuAng > 35){
                fL.setPower(0.5 );
                fR.setPower(-0.5 );
                bL.setPower(0.5 );
                bR.setPower(-0.5);
            }else if(vuAng - ang > 35){
                fL.setPower(-0.5 );
                fR.setPower(0.5 );
                bL.setPower(-0.5 );
                bR.setPower(0.5 );
            }else if (ang < vuAng) {
                fL.setPower(-0.25 );
                fR.setPower(0.25);
                bL.setPower(-0.25 );
                bR.setPower(0.25 );
            }else if (ang > vuAng) {
                fL.setPower(0.25 );
                fR.setPower(-0.25 );
                bL.setPower(0.25);
                bR.setPower(-0.25);
            }
            ang = getHeading();
            turned = (Math.abs(ang - vuAng) <= 0.5);
        }
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int rundetect(SkystoneDetector sky){
        double dist = sky.getDist();
        int position = 0;
        if(dist < 150||dist > 380){
            position = 0;
        }else if(dist < 320){
            position = 1;
        }else{
            position = 0;
        }
        return position;
    }


}
