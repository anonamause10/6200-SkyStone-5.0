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
import com.qualcomm.robotcore.hardware.DigitalChannel;
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

@Autonomous(name="re", group="ree")

public class rSauto extends LinearOpMode
{
    private int[] zeroPos = {0,0,0,0};
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
    private DistanceSensor sRR;
    private DistanceSensor sRL;
    private DigitalChannel touch;
    private PIDController pidDrive;
    private DcMotor YEETER;

    private double voltage = 0.0;
    private double scale = 0.0;
    private int blockPos = 1;
    int obligitoryCounter = 0;
    int obligitoryCounter2 = 0;
    PIDController           pidRotate;
    Orientation             lastAngles = new Orientation();
    double globalAngle, rotation;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    boolean usingCamera = true;
    SkystoneDetectorNew detector = null;

    @Override
    public void runOpMode() {

        detector = new SkystoneDetectorNew(hardwareMap, true, true,true);

        voltage = getBatteryVoltage();
        scale = 12.8 / voltage;
        double scale2 = 1;
        if (voltage<=13.4){
            scale2 = 1.15;
            scale = scale*scale2;
        }

        // create a sound parameter that holds the desired player parameters.
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        pidRotate = new PIDController(.003, .00003, 0);
        pidDrive = new PIDController(.05, 0, 0);
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, 1.0);
        pidDrive.setInputRange(-270, 270);
        pidDrive.enable();

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
        ROTATE.setPosition(.69);
        servo = hardwareMap.get(Servo.class, "left");
        servo2 = hardwareMap.get(Servo.class, "right");
        servo.setPosition(0);
        servo2.setPosition(.4);

        sR = hardwareMap.get(DistanceSensor.class, "DSB");
        sR2 = hardwareMap.get(DistanceSensor.class, "DS2");
        sRR = hardwareMap.get(DistanceSensor.class, "DSR");
        sRL = hardwareMap.get(DistanceSensor.class, "DSL");
        sRF = hardwareMap.get(DistanceSensor.class, "DSF");
        touch = hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);


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

        YEETER = hardwareMap.get(DcMotor.class, "YEET");


        telemetry.addData("Robot", "Initialized");
        int blockPos = 0;

        //waitForStart();
        while (!isStopRequested()&&!opModeIsActive()) {
            if(usingCamera)
                telemetry.addData("blockPos", detector.getPos());
            else
                telemetry.addData("blockPOs", blockPos);
            if(gamepad1.x){blockPos = 0; usingCamera = false; detector.stop();}
            if(gamepad1.a){blockPos = 1; usingCamera = false; detector.stop();}
            if(gamepad1.b){blockPos = 2; usingCamera = false; detector.stop();}

            telemetry.update();
        }
        if(usingCamera)
            blockPos = (int)detector.getPos();

        runtime.reset();

        double[] array1 = {0.2*scale, 0.68*scale, 0.2*scale, 0.68*scale};

        if(blockPos == 0){
            double[] array2 = {0.89*scale, 0.1*scale, 0.89*scale, 0.1*scale};
            array1 = array2;
        }else if(blockPos == 2){
            double[] array2 = {0.1*scale, 0.85*scale, 0.1*scale, 0.85*scale};
            array1 = array2;
        }
        if(blockPos == 2)
            goV2(450, 0.5*scale, array1, false);
        else
            goV2(850, 0.5*scale, array1, false);
        intake();
        if(blockPos==2)
            sleep(100);
        sleep(300);

        double power = 0.3*scale;
        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);
        if(blockPos!=1)
            sleep(400);
        sleep(500);

        if(sR2.getDistance(DistanceUnit.MM)<70){
            intakeOff();
            closeClaw();}

        power = -0.5*scale;
        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);
        sleep(80);
        if(CLAW.getPosition()>0.1){
        intakeOff();
        closeClaw();
        }

        sleep(50);

        if(blockPos == 2)
            sleep(570);
        else if(blockPos == 1)
            sleep(690);
        else
            sleep(780);

        array1 = new double[] {0.5, 0.5, 0.5, 0.5};

        turn(270, new double[]{0,0,0,0}, false, 0);
        servosUp();

        driveSleep(1000,0.7);
        moveWithForwardSensor(800, 0.6*scale, true);
        LIFT.setPower(0.2);
        turn(180, new double[] {0,0,0,0}, false, 2);
        rotateOut();
        goV2(-530, 0.3, new double[] {-0.25, -0.25, -0.25, -0.25}, true);
        servosDown();
        fL.setPower(0.12*scale);
        fR.setPower(0.12*scale);
        bL.setPower(0.12*scale);
        bR.setPower(0.12*scale);
        openClaw();
        sleep(400);

        /**moveWithForwardSensor(680, 0.7, true);
         if(CLAW.getPosition()>0.1)
         rotateIn();
         array1 = new double[] {0,0,0,0};
         turn(270, array1, true, 1);*/
        fL.setPower(0.7*scale);
        fR.setPower(-0.7*scale);
        bL.setPower(-0.7*scale);
        bR.setPower(0.7*scale);
        sleep(300);
        rotateIn();
        fL.setPower(0.7*scale);
        fR.setPower(0.7*scale);
        bL.setPower(0.7*scale);
        bR.setPower(0.7*scale);
        sleep(800);
        fL.setPower(0.7*scale);
        fR.setPower(-0.7*scale);
        bL.setPower(0.7*scale);
        bR.setPower(-0.7*scale);

        while(opModeIsActive()&&getHeading()>91){
            if(getHeading()<125 && ROTATE.getPosition()>0.6 && LIFT.getCurrentPosition()>5){
                LIFT.setPower(-0.2*scale);
            }else if(getHeading()<125){
                LIFT.setPower(-0.01*scale);
            }
            updateT();
        }
        motorsOff();
        servosUp();
        LIFT.setPower(-0.01*scale);

        power = 0.3*scale;
        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);
        sleep(10);

        turn(90, new double[]{0,0,0,0}, getHeading()<90, 0);

        if(sRL.getDistance(DistanceUnit.MM)<1128)
            moveWithLeftSensor(1130, 0.3*scale);
        if(sRL.getDistance(DistanceUnit.MM)>1132)
            moveWithLeftSensor(1130, 0.3*scale);
        LIFT.setPower(-0.02);
        turn(90, new double[]{0,0,0,0}, 90>getHeading(), 0);


        driveSleep(1000, 0.7);
        array1 = new double[]{0.95*scale, 0.25*scale, 0.95*scale, 0.25*scale};

        if(Math.abs(getHeading()-90)>1)
            turn(90, new double[]{0,0,0,0}, 90>getHeading(), 0);


        if(blockPos == 0)
            moveWithForwardSensor(1300, 0.3*scale, false);
        else if(blockPos == 2)
            moveWithForwardSensor(860, 0.4*scale, false);
        else
            moveWithForwardSensor(1030, 0.35*scale, false);

        if(sRL.getDistance(DistanceUnit.MM)<1130)
            moveWithLeftSensor(1133, 0.4*scale);

        fL.setPower(array1[0]);
        fR.setPower(array1[1]);
        bL.setPower(array1[2]);
        bR.setPower(array1[3]);
        intake();
        sleep(1100);

        power = -0.5*scale;
        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);
        if(blockPos!=2)
            sleep(200);
        sleep(450);
        intakeOff();
        closeClaw();

        turn(90, new double[]{0,0,0,0}, true, 0);

        if(sRL.getDistance(DistanceUnit.MM)<1130)
            moveWithLeftSensor(1133, 0.3*scale);
        if(sRL.getDistance(DistanceUnit.MM)>1135){
            moveWithLeftSensor(1133, 0.3*scale);
            if(sRL.getDistance(DistanceUnit.MM)<1130)
                moveWithLeftSensor(1133, 0.3*scale);
        }
        else
            turn(90, new double[]{0,0,0,0}, 90>getHeading(), 0);


        driveSleep(1350, -0.7);
        driveSleep(720, -0.55);

        LIFT.setPower(0.9);
        power = -0.5*scale;
        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);
        sleep(300);

        rotateOut();
        LIFT.setPower(0.2);
        power = -0.3*scale;
        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);
        sleep(120);


        sleep(150);
        if(sR.getDistance(DistanceUnit.MM)>100)
            sleep(200);
        motorsOff();
        sleep(300);
        openClaw();
        sleep(200);
        if(getHeading()<92){
            turn(95, new double[]{0,0,0,0}, 92>getHeading(), 1);
        }

        YEETER.setPower(-1);
        power = 0.3*scale;
        fL.setPower(power);
        fR.setPower(power + 0.01);
        bL.setPower(power);
        bR.setPower(power + 0.01);

        sleep(300);
        YEETER.setPower(0);
        rotateIn();
        sleep(600);
        motorsOff();

        LIFT.setPower(-0.3);

        if(usingCamera)
            detector.stop();
        if(LIFT.getCurrentPosition()<10 || !touch.getState()){
            LIFT.setPower(0);
        }
        sleep(300);
        if(LIFT.getCurrentPosition()<10 || !touch.getState()){
            LIFT.setPower(0);
        }
        //goV2(1000, 0.7, new double[]{0,0,0,0}, true);
        servosDown();

        while(opModeIsActive()){
        if(LIFT.getCurrentPosition()<10 || !touch.getState()){
            LIFT.setPower(0);
            break;
        }}
    }
    private void goV2(int ticks, double power, double[] endPowers, boolean intakeDeployed){
        boolean phase2 = false;
        resetEncoders();
        if(!intakeDeployed){
            LIFT.setPower(0.7);
            runtim2.reset();
        }
        if(ticks < 0){
            fL.setPower(-power);
            fR.setPower(-power);
            bL.setPower(-power);
            bR.setPower(-power);
            while(opModeIsActive()&&averageTicks()>ticks){
                updateT();
            }
        }else{
            fL.setPower(power);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(power);
            while(opModeIsActive()&&averageTicks()<ticks){
                if((!intakeDeployed && LIFT.getCurrentPosition()>300) || runtim2.seconds()>0.7){
                    LIFT.setPower(-0.7);
                    phase2 = true;
                    runtim2.reset();
                }
                if((phase2 && (!touch.getState() || LIFT.getCurrentPosition() < -40)) || runtim2.seconds()>0.7){
                    LIFT.setPower(0);
                    LIFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    LIFT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intakeDeployed = true;
                    phase2 = false;
                }
                if(LIFT.getCurrentPosition()>=50 && intakeDeployed){
                    LIFT.setPower(-0.4*scale);
                }else if(intakeDeployed){
                    LIFT.setPower(0);
                }
                if(sR2.getDistance(DistanceUnit.MM)<70){
                    closeClaw();
                    intakeOff();
                }
                updateT();
            }
        }
        fL.setPower(endPowers[0]);
        fR.setPower(endPowers[1]);
        bL.setPower(endPowers[2]);
        bR.setPower(endPowers[3]);
    }


    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle+360)%360;
    }
    private void rotateIn(){
        ROTATE.setPosition(0.69);
    }
    private void rotateOut(){
        ROTATE.setPosition(0.025);
    }
    private void closeClaw(){
        CLAW.setPosition(0);
    }
    private void openClaw(){
        CLAW.setPosition(0.15);
    }
    private void servosUp(){
        servo.setPosition(.4);
        servo2.setPosition(.65);
        sleep(100);
    }
    private void servosDown(){
        servo.setPosition(0);
        servo2.setPosition(.4);
        sleep(100);
    }

    private void intake(){
        IN1.setPower(0.7);
        IN2.setPower(0.7);
    }

    private void intakeOff(){
        IN1.setPower(0);
        IN2.setPower(0);
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
        resetEncoders();
    }

    void moveWithForwardSensor(int target, double power, boolean placingblock){

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtim2.reset();
        if(sRF.getDistance(DistanceUnit.MM)>target){
            while(opModeIsActive()&&(sRF.getDistance(DistanceUnit.MM) > target)&&runtim2.seconds()<5){
                if(placingblock){
                    placeBlock(sR.getDistance(DistanceUnit.MM)<100);
                }
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
        motorsOff();
    }
    void moveWithRightSensor(int target, double power){

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtim2.reset();
        double startAngle = getHeading();
        if(sRR.getDistance(DistanceUnit.MM)<target){
            while(opModeIsActive()&&(sRR.getDistance(DistanceUnit.MM) < target)&&runtim2.seconds()<1){
                fL.setPower(-power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(-power);

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
            }}else {
            while (opModeIsActive() && (sRR.getDistance(DistanceUnit.MM) > target) && runtim2.seconds() < 1) {
                fL.setPower(power);
                fR.setPower(-power);
                bL.setPower(-power);
                bR.setPower(power);
                if (startAngle == 0) {
                    if (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle < 0.25) {
                        fL.setPower(fL.getPower() - 0.02);
                        fR.setPower(fR.getPower() + 0.02);
                        bL.setPower(bL.getPower() - 0.02);
                        bR.setPower(bR.getPower() + 0.02);
                    } else if ((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle > 0.25)) {
                        fL.setPower(fL.getPower() + 0.02);
                        fR.setPower(fR.getPower() - 0.02);
                        bL.setPower(bL.getPower() + 0.02);
                        bR.setPower(bR.getPower() - 0.02);
                    } else {
                    }
                } else {
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
                    } else {
                    }
                }
                updateT();

            }
        }
        motorsOff();
    }
    void moveWithLeftSensor(int target, double power){
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtim2.reset();
        double startAngle = getHeading();
        if(sRL.getDistance(DistanceUnit.MM)<target){
            while(opModeIsActive()&&(sRL.getDistance(DistanceUnit.MM) < target)&&runtim2.seconds()<2){
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
            while(opModeIsActive()&&(sRL.getDistance(DistanceUnit.MM) > target)&&runtim2.seconds()<2){
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
        motorsOff();
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


    void turn(double tun, double[] endPowers, boolean targGreater, int foundation){

        double vuAng = tun;
        boolean turned = false;
        runtim2.reset();
        while (!turned && opModeIsActive() && runtim2.seconds() < 4) {
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
            }else if(ang>=260&& vuAng<=90){
                fL.setPower(-0.4);
                fR.setPower(0.4);
                bL.setPower(-0.4);
                bR.setPower(0.4);
            }else if(ang<=90&& vuAng>=260){
                fL.setPower(0.4);
                fR.setPower(-0.4);
                bL.setPower(0.4);
                bR.setPower(-0.4);
            }else if (ang-vuAng > 35){
                if(foundation==1) {
                    fL.setPower(0.7);
                    fR.setPower(-0.7);
                    bL.setPower(0.7);
                    bR.setPower(-0.7);
                }else{
                    fL.setPower(0.4);
                    fR.setPower(-0.4);
                    bL.setPower(0.4);
                    bR.setPower(-0.4);
                }
            }else if(vuAng - ang > 35){
                if(foundation==1) {
                    fL.setPower(-0.7);
                    fR.setPower(0.7);
                    bL.setPower(-0.7);
                    bR.setPower(0.7);
                }else{
                    fL.setPower(-0.4);
                    fR.setPower(0.4);
                    bL.setPower(-0.4);
                    bR.setPower(0.4);
                }
            }else if (ang < vuAng) {
                if(!targGreater){
                    turned = true;
                    motorsOff();
                }else{
                    if(foundation==1){
                        fL.setPower(-0.4*scale);
                        fR.setPower(0.4*scale);
                        bL.setPower(-0.4*scale);
                        bR.setPower(0.4*scale);
                    }else if(foundation==2) {
                        fL.setPower(-0.19*scale);
                        fR.setPower(0.19*scale);
                        bL.setPower(-0.19*scale);
                        bR.setPower(0.19*scale);
                    }else{
                        fL.setPower(-0.155*scale);
                        fR.setPower(0.155*scale);
                        bL.setPower(-0.155*scale);
                        bR.setPower(0.155*scale);
                    }
                }
            }else if (ang > vuAng) {
                if(targGreater){
                    turned = true;
                    motorsOff();
                }else{
                    if(foundation==1) {
                        fL.setPower(0.4*scale);
                        fR.setPower(-0.4*scale);
                        bL.setPower(0.4*scale);
                        bR.setPower(-0.4*scale);
                    }else if(foundation==2){
                        fL.setPower(0.19*scale);
                        fR.setPower(-0.19*scale);
                        bL.setPower(0.19*scale);
                        bR.setPower(-0.19*scale);
                    }else{
                        fL.setPower(0.155*scale);
                        fR.setPower(-0.155*scale);
                        bL.setPower(0.155*scale);
                        bR.setPower(-0.155*scale);
                    }}
            }
            ang = getHeading();
            if(!turned)
                turned = (Math.abs(ang - vuAng) <= 0.2);
        }
        fL.setPower(endPowers[0]);
        fR.setPower(endPowers[1]);
        bL.setPower(endPowers[2]);
        bR.setPower(endPowers[3]);
    }

    private void resetEncoders(){
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void motorsOff(){
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }
    private double averageTicks(){
        return (fL.getCurrentPosition()+fR.getCurrentPosition()+bL.getCurrentPosition()+bR.getCurrentPosition())/4;
    }
    private void strafeToAngle(int angle, double power){
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtim2.reset();
        boolean working = true;
        if(getHeading() < 30 && angle > 300){
            fL.setPower(power);
            fR.setPower(-power);
            bL.setPower(-power);
            bR.setPower(power);
            while (opModeIsActive() && runtim2.seconds() <= 4 && working) {
                if(sR.getDistance(DistanceUnit.MM)<100){
                    placeBlock(true);
                }
                updateT();
                if (getHeading() < angle && getHeading() > (angle - 10)) {
                    working = false;
                }
            }
        }else if(angle > getHeading()) {
            fL.setPower(-power);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(-power);

            while (opModeIsActive() && runtim2.seconds() <= 4 && working) {
                if(sR.getDistance(DistanceUnit.MM)<100){
                    placeBlock(true);
                }
                updateT();
                if (getHeading() > angle) {
                    working = false;
                }
            }
        }else if(angle < getHeading()) {
            fL.setPower(power);
            fR.setPower(-power);
            bL.setPower(-power);
            bR.setPower(power);
            while (opModeIsActive() && runtim2.seconds() <= 4 && working) {
                if(sR.getDistance(DistanceUnit.MM)<100){
                    placeBlock(true);
                }
                updateT();
                if (getHeading() < angle) {
                    working = false;
                }
            }
        }
    }

    public void drive(int ticks, double power){
        // restart imu angle tracking.
        resetAngle();
        int degrees = 0;

        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
                if(LIFT.getCurrentPosition()>=50){
                    LIFT.setPower(-0.4*scale);
                }else{
                    LIFT.setPower(0);
                }
                pow = pidDrive.performPID(getAngle()); // power will be + on left turn.
                updateT();
                fL.setPower(power-pow);
                fR.setPower(power+pow);
                bL.setPower(power-pow);
                bR.setPower(power+pow);
                if (Math.abs((4*ticks)-fL.getCurrentPosition()-fR.getCurrentPosition()-fL.getCurrentPosition()-bR.getCurrentPosition())<100){
                    working = false;
                }
            } while (opModeIsActive() && runtim2.seconds()<1.5 && working );

        // turn the motors off.
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

        rotation = getAngle();

        // wait for rotation to stop.

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void driveSleep(int time, double power){
        // restart imu angle tracking.
        resetAngle();
        int degrees = 0;

        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
            if(LIFT.getCurrentPosition()>=50){
                LIFT.setPower(-0.4*scale);
            }else{
                LIFT.setPower(0);
            }
            pow = pidDrive.performPID(getAngle()); // power will be + on left turn.
            updateT();
            fL.setPower(power-pow);
            fR.setPower(power+pow);
            bL.setPower(power-pow);
            bR.setPower(power+pow);
        } while (opModeIsActive() && runtim2.milliseconds()<time);

        // turn the motors off.
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

        rotation = getAngle();

        // wait for rotation to stop.

        // reset angle tracking on new heading.
        resetAngle();
    }

    private void placeBlock(boolean readyToPlace){

        if(CLAW.getPosition()>=0.1 && LIFT.getCurrentPosition()>0) {
            if(ROTATE.getPosition()>=0.5 && obligitoryCounter2 >= 13)
                LIFT.setPower(-0.4);
            else{
                if(obligitoryCounter2 == 0) {
                    rotateIn();
                    LIFT.setPower(0.2);
                    obligitoryCounter2++;
                }else{
                    obligitoryCounter2++;
                    LIFT.setPower(0.2);
                }
            }
        }else if(LIFT.getCurrentPosition()>=550){
            LIFT.setPower(0.2);
            rotateOut();
        }else if(CLAW.getPosition()<0.1 && ROTATE.getPosition()>=0.5 && LIFT.getCurrentPosition()<570){
            LIFT.setPower(0.7);
        }else{
            LIFT.setPower(0.2);
        }

        if(ROTATE.getPosition()==0.025){
            if(obligitoryCounter > -1 && obligitoryCounter<10 && CLAW.getPosition() < 0.1){
                obligitoryCounter++;
            }else if(readyToPlace){
                openClaw();
                obligitoryCounter = -1;
                rotateIn();
            }
        }
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