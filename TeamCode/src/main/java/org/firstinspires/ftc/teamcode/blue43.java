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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
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

@Autonomous(name="blue43", group="ree")


public class blue43 extends LinearOpMode
{
    private Servo blockPusher = null;

    boolean blockPushed = false;
    double starttime = 0;
    private int[] zeroPos = {0,0,0,0};
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtim2 = new ElapsedTime();
    private boolean liftgoingup = false;
    private boolean imuworking = true;
    private DcMotorEx fL = null;
    private DcMotorEx fR = null;
    private DcMotorEx bL = null;
    private DcMotorEx bR = null;
    private DcMotor IN1 = null;
    private DcMotor IN2 = null;
    private DcMotor LIFT = null;
    private Servo CLAW =null;
    private Servo ROTATE = null;
    private Servo servo = null;
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
    boolean sensorWorking = true;
    boolean intFirst = false;
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    boolean usingCamera = true;
    boolean frontIsWorking = true;
    //SkystoneDetectorNew detector = null;
    int distanceFromWall = 683;
    boolean failedOnBlockSide = false;

    @Override
    public void runOpMode() throws InterruptedException {



        voltage = getBatteryVoltage();
        scale = 13.2 / voltage;

        // create a sound parameter that holds the desired player parameters.
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        pidRotate = new PIDController(.003, .00003, 0);
        pidDrive = new PIDController(.02, 0.005, 0);
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, 1.0);
        pidDrive.setInputRange(-270, 270);
        pidDrive.enable();

        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;
        fL = hardwareMap.get(DcMotorEx.class, "fL");
        fR = hardwareMap.get(DcMotorEx.class, "fR");
        bL = hardwareMap.get(DcMotorEx.class, "bL");
        bR = hardwareMap.get(DcMotorEx.class, "bR");
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
        YEETER = hardwareMap.get(DcMotor.class, "YEET");

        LIFT = hardwareMap.get(DcMotor.class, "LIFT");
        LIFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IN1 = hardwareMap.get(DcMotor.class, "IN1");
        IN1.setDirection(DcMotor.Direction.FORWARD);
        IN2 = hardwareMap.get(DcMotor.class, "IN2");
        IN2.setDirection(DcMotor.Direction.REVERSE);
        CLAW = hardwareMap.get(Servo.class, "CLAW");
        CLAW.setPosition(.2);
        ROTATE = hardwareMap.get(Servo.class, "ROTATE");
        ROTATE.setPosition(.84);
        servo = hardwareMap.get(Servo.class, "left");

        touch = hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);
        blockPusher = hardwareMap.get(Servo.class, "push");
        blockPusher.setPosition(0);


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
        SkystoneDetectorNew detector = new SkystoneDetectorNew(hardwareMap, true, false,true);
        telemetry.addData("Robot", "Initialized");
        int blockPos = 0;

        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            if (usingCamera) {
                blockPos = (int) detector.getPos();
                telemetry.addData("camera", blockPos);
            }
            else
                telemetry.addData("blockPOs", blockPos);
            if (gamepad1.x && gamepad1.right_bumper) {
                blockPos = 0;
                usingCamera = false;
                //detector.stop();
            }
            if (gamepad1.a && gamepad1.right_bumper) {
                blockPos = 1;
                usingCamera = false;
                //detector.stop();
            }
            if (gamepad1.b &&gamepad1.right_bumper) {
                blockPos = 2;
                usingCamera = false;
            }
            telemetry.update();
        }
        if(usingCamera)
        blockPos = (int)detector.getPos();
        goV3(720, 0.7);
        servosUp();
        if(blockPos==0){
            strafeLeft(300, 0.7, new double[]{0,0,0,0});
            if(Math.abs(getHeading0())>=0.1) {
                turnToZero();
            }
        }else if(blockPos==2){
            strafeRight(630, 0.7, new double[]{0,0,0,0});
            if(Math.abs(getHeading0())>=0.1) {
                turnToZero();
            }
        }else{
            strafeRight(350, 0.7, new double[]{0,0,0,0});
            if(Math.abs(getHeading0())>=0.1) {
                turnToZero();
            }
        }
        intake();
        goV3(800, 0.5);
        if(Math.abs(getHeading0())>=0.1){
            turnToZero();
        }
        if(blockPos==0)
            goV3(-680, 0.7);
        else if(blockPos==1)
            goV3(-750, 0.7);
        else
            goV3(-760, 0.7);
        blockPusher.setPosition(0.6);
        starttime = runtime.milliseconds();
        blockPushed = true;
        intakeOff();
        turn(88, true, 0);

        if(blockPos==2)
            drive(4000, 0.7);
        else if(blockPos==1)
            drive(3700,0.7);
        else
            drive(3300, 0.7);
        blockPusher.setPosition(0.9);
        LIFT.setPower(1);
        turn(178, new double[]{0,0,0,0}, true, 0);
        LIFT.setPower(0.2);
        if(blockPos==2)
            moveWhileUsingLift(-450, 0.29, 500, 0.4, new double[] {0,0,0,0});
        else
            moveWhileUsingLift(-350, 0.29, 500, 0.4, new double[] {0,0,0,0});
        LIFT.setPower(0.2);
        rotateOut();
        sleep(800);
        openClaw();
        sleep(200);
        blockPusher.setPosition(0);
        rotateIn();
        servosDown();
        sleep(500);
        fL.setPower(-0.7 * scale);
        fR.setPower(0.7 * scale);
        bL.setPower(0.7 * scale);
        bR.setPower(-0.7 * scale);
        sleep(300);
        fL.setPower(0.7 * scale);
        fR.setPower(0.7 * scale);
        bL.setPower(0.7 * scale);
        bR.setPower(0.7 * scale);
        sleep(470);
        fL.setPower(-0.7 * scale);
        fR.setPower(0.7 * scale);
        bL.setPower(-0.7 * scale);
        bR.setPower(0.7 * scale);
        while (opModeIsActive() && getHeading() < 270) {
            updateT();
        }
        motorsOff();
        servosUp();
        LIFT.setPower(-0.4);
        goV3(-430, 0.4);
        strafeLeft(490, 0.5, new double[]{0,0,0,0});
        adjust270();
        if(blockPos==2)
            drive(3750, 0.7);
        else if(blockPos==1)
            drive(3500, 0.7);
        else
            drive(3150, 0.7);

        adjust270();
        strafeLeft(820, 0.7, new double[]{0,0,0,0});
        intake();
        goV3(250, 0.3);
        strafeRight(800, 0.7, new double[]{0,0,0,0});
        adjust270();
        blockPusher.setPosition(0.6);
        starttime = runtime.milliseconds();
        blockPushed = true;
        intakeOff();
        if(blockPos==2)
            goV3(-4100, 0.7);
        else if(blockPos==1)
            goV3(-3800, 0.7);
        else
            goV3(-3400, 0.7);
        blockPusher.setPosition(0.9);
        moveWhileUsingLift(-350, 0.5, 500, 0.9, new double[] {0,0,0,0});
        LIFT.setPower(0.2);
        rotateOut();
        sleep(900);
        openClaw();
        YEETER.setPower(-1);
        sleep(300);
        YEETER.setPower(0);
        rotateIn();
        goV3(200, 0.5);
        sleep(100);
    }

    private void goV3(int ticks, double power, double[] x){
        boolean phase2 = false;
        resetEncoders();
        runtim2.reset();
        if(ticks < 0){
            if(Math.abs(ticks - averageTicks2()) < ticks/4 ){
                fL.setPower(-power/2);
                fR.setPower(-power/2);
                bL.setPower(-power/2);
                bR.setPower(-power/2);
            }else {
                fL.setPower(-power);
                fR.setPower(-power);
                bL.setPower(-power);
                bR.setPower(-power);
            }
            while(opModeIsActive()&&averageTicks2()>ticks){
                updateT();
                if(blockPushed && runtime.milliseconds()>=starttime + 400){
                    CLAW.setPosition(0.03);
                    blockPushed = false;
                }
                if((!touch.getState()||LIFT.getCurrentPosition()<5) && LIFT.getPower()<0 ) {
                    LIFT.setPower(0);
                }
            }
        }else{
            if(ticks - averageTicks2() < ticks/4 ){
                fL.setPower(power/2);
                fR.setPower(power/2);
                bL.setPower(power/2);
                bR.setPower(power/2);
            }else {
                fL.setPower(power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(power);
            }
            while(opModeIsActive()&&averageTicks2()<ticks){
                updateT();
                if(blockPushed && runtime.milliseconds()>=starttime + 400){
                    CLAW.setPosition(0.03);
                    blockPushed = false;
                }
                if((!touch.getState()||LIFT.getCurrentPosition()<5) && LIFT.getPower()<0 ){
                    LIFT.setPower(0);
                }
            }
        }
        fL.setPower(x[0]);
        fR.setPower(x[1]);
        bL.setPower(x[2]);
        bR.setPower(x[3]);
    }
    private void goV3(int ticks, double power){
        goV3(ticks, power, new double[]{0,0,0,0});
    }
    private void goV3SecondBlock(int ticks, double power){
        resetEncoders();
        runtim2.reset();
        if(ticks < 0){
            if(Math.abs(ticks - averageTicks2()) < ticks/4 ){
                fL.setPower(-power/2);
                fR.setPower(-power/2);
                bL.setPower(-power/2);
                bR.setPower(-power/2);
            }else {
                fL.setPower(-power);
                fR.setPower(-power);
                bL.setPower(-power);
                bR.setPower(-power);
            }
            while(opModeIsActive()&&averageTicks2()>ticks){
                if(averageTicks2() < ticks + 1300 ) {
                    if (LIFT.getCurrentPosition() < 450 && rotatedIN()){
                        LIFT.setPower(1);
                    }else{
                        LIFT.setPower(0);
                        rotateOut();
                    }
                }
                updateT();
            }
        }else{
            if(ticks - averageTicks2() < ticks/4 ){
                fL.setPower(power/2);
                fR.setPower(power/2);
                bL.setPower(power/2);
                bR.setPower(power/2);
            }else {
                fL.setPower(power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(power);
            }
            while(opModeIsActive()&&averageTicks2()<ticks){
                updateT();
            }
        }
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }
    private int averageTicks2(){
        return (bL.getCurrentPosition() + bR.getCurrentPosition()) /2;
    }
    private void strafeRight(int ticks, double power, double[] endPowers){
        resetEncoders();
        fL.setPower(power);
        fR.setPower(-power);
        bL.setPower(-power);
        bR.setPower(power);
        while(opModeIsActive()&&fL.getCurrentPosition()<ticks){
            updateT();
        }
        fL.setPower(endPowers[0]);
        fR.setPower(endPowers[1]);
        bL.setPower(endPowers[2]);
        bR.setPower(endPowers[3]);
    }
    private void strafeLeft(int ticks, double power, double[] endPowers){
        resetEncoders();
        fL.setPower(-power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(-power);
        while(opModeIsActive()&&fL.getCurrentPosition()>(-ticks)){
            updateT();
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
    public double getHeading0() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle);
    }
    private void rotateIn(){
        ROTATE.setPosition(0.84);
    }
    private boolean rotatedIN(){
        return ROTATE.getPosition()>0.5;
    }
    private void rotateOut(){
        ROTATE.setPosition(0.16);
    }
    private void closeClaw(){
        CLAW.setPosition(0.05);
    }
    private void openClaw(){
        CLAW.setPosition(0.2);
    }
    private void servosUp(){
        servo.setPosition(0.85);
    }
    private void servosDown(){
        servo.setPosition(0.4);
    }

    private void intake(){
        IN1.setPower(0.45);
        IN2.setPower(0.45);
    }
    private void outtake(){
        IN1.setPower(-0.4);
        IN2.setPower(-0.4);
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
        //telemetry.addData("Right Sensor", sRR.getDistance(DistanceUnit.MM));
        telemetry.addData("Sensor Working", sensorWorking);
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

    void turn(double tun, int foundation){
        turn(tun, new double[] {0,0,0,0}, tun > getHeading(), foundation);
    }
    void turn(double tun, boolean targ, int foundation){
        turn(tun, new double[] {0,0,0,0}, targ, foundation);
    }
    void turn(double tun, double[] endPowers, boolean targGreater, int foundation){
        boolean startedAtZero = getHeading()==0;
        double vuAng = tun;
        boolean turned = false;
        runtim2.reset();
        while (!turned && opModeIsActive() && runtim2.seconds() < 3) {
            if(runtim2.seconds()>2 && getHeading() == 0 && startedAtZero){
                imuworking = false;
            }
            if(LIFT.getCurrentPosition()>400&&LIFT.getPower()==1){
                LIFT.setPower(0.2);
            }
            if(blockPushed && runtime.milliseconds()>=starttime + 400){
                CLAW.setPosition(0.03);
                blockPushed = false;
            }
            double ang = getHeading();

            telemetry.addData("Angle", ang);
            telemetry.addData("TurnTo", vuAng);
            telemetry.addData("Wheel Power", "front left (%.2f), front right (%.2f), " +
                            "back left (%.2f), back right (%.2f)", fL.getPower(), fR.getPower(),
                    bL.getPower(), bR.getPower());
            telemetry.update();
            double smallPower = 280;
            double bigPower = 1500;
            double mediumPower = 1000;
            if(Math.abs(ang - vuAng) <= 0.7){
                fL.setPower(0);
                fR.setPower(0);
                bL.setPower(0);
                bR.setPower(0);
            }else if(ang>=260&& vuAng<=90){
                fL.setPower(-mediumPower);
                fR.setPower(mediumPower);
                bL.setPower(-mediumPower);
                bR.setPower(mediumPower);
            }else if(ang<=90&& vuAng>=260){
                fL.setVelocity(mediumPower);
                fR.setVelocity(-mediumPower);
                bL.setVelocity(mediumPower);
                bR.setVelocity(-mediumPower);
            }else if (ang-vuAng > 35){
                if(foundation==1) {
                    fL.setVelocity(bigPower);
                    fR.setVelocity(-bigPower);
                    bL.setVelocity(bigPower);
                    bR.setVelocity(-bigPower);
                }else{
                    fL.setVelocity(mediumPower);
                    fR.setVelocity(-mediumPower);
                    bL.setVelocity(mediumPower);
                    bR.setVelocity(-mediumPower);
                }
            }else if(vuAng - ang > 35){
                if(foundation==1) {
                    fL.setVelocity(-bigPower);
                    fR.setVelocity(bigPower);
                    bL.setVelocity(-bigPower);
                    bR.setVelocity(bigPower);
                }else{
                    fL.setVelocity(-mediumPower);
                    fR.setVelocity(mediumPower);
                    bL.setVelocity(-mediumPower);
                    bR.setVelocity(mediumPower);
                }
            }else if (ang < vuAng) {
                if(!targGreater){
                    turned = true;
                    motorsOff();
                }else{
                    if(foundation==1){
                        fL.setVelocity(-mediumPower);
                        fR.setVelocity(mediumPower);
                        bL.setVelocity(-mediumPower);
                        bR.setVelocity(mediumPower);
                    }else if(foundation==2) {
                        fL.setVelocity(-smallPower);
                        fR.setVelocity(smallPower);
                        bL.setVelocity(-smallPower);
                        bR.setVelocity(smallPower);
                    }else{
                        fL.setVelocity(-smallPower);
                        fR.setVelocity(smallPower);
                        bL.setVelocity(-smallPower);
                        bR.setVelocity(smallPower);
                    }
                }
            }else if (ang > vuAng) {
                if(targGreater){
                    turned = true;
                    motorsOff();
                }else{
                    if(foundation==1) {
                        fL.setVelocity(mediumPower);
                        fR.setVelocity(-mediumPower);
                        bL.setVelocity(mediumPower);
                        bR.setVelocity(-mediumPower);
                    }else if(foundation==2){
                        fL.setVelocity(smallPower*1.2);
                        fR.setVelocity(-smallPower*1.2);
                        bL.setVelocity(smallPower*1.2);
                        bR.setVelocity(-smallPower*1.2);
                    }else{
                        fL.setVelocity(smallPower);
                        fR.setVelocity(-smallPower);
                        bL.setVelocity(smallPower);
                        bR.setVelocity(-smallPower);
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
    private void turnToZero(){
        boolean turned = false;
        double smallPower = 240;
        runtim2.reset();
        while (!turned && opModeIsActive() && runtim2.seconds() < 0.7) {

            if (Math.abs(getHeading0()) <= 0.1) {
                motorsOff();
                turned = true;
            } else if (getHeading0() > 0) {
                fL.setVelocity(smallPower);
                fR.setVelocity(-smallPower);
                bL.setVelocity(smallPower);
                bR.setVelocity(-smallPower);
            } else {
                fL.setVelocity(-smallPower);
                fR.setVelocity(smallPower);
                bL.setVelocity(-smallPower);
                bR.setVelocity(smallPower);
            }
        }
    }
    private void adjust270(){
        boolean turned = false;
        double smallPower = 240;
        runtim2.reset();
        while (!turned && opModeIsActive() && runtim2.seconds() < 0.7) {
            if(blockPushed && runtime.milliseconds()>=starttime + 400){
                CLAW.setPosition(0.03);
                blockPushed = false;
            }
            if (Math.abs(getHeading()-270) <= 0.1) {
                motorsOff();
                turned = true;
            } else if (getHeading() > 270) {
                fL.setVelocity(smallPower);
                fR.setVelocity(-smallPower);
                bL.setVelocity(smallPower);
                bR.setVelocity(-smallPower);
            } else {
                fL.setVelocity(-smallPower);
                fR.setVelocity(smallPower);
                bL.setVelocity(-smallPower);
                bR.setVelocity(smallPower);
            }
        }
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

    public void drive(int ticks, double power){
        // restart imu angle tracking.
        resetAngle();
        int degrees = 0;
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);


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
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                fL.setPower(power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(power);
                //sleep(100);
                if(blockPushed && runtime.milliseconds()>=starttime + 400){
                    CLAW.setPosition(0.03);
                    blockPushed = false;
                }
                if((!touch.getState()||LIFT.getCurrentPosition()<5) && LIFT.getPower()<0 ){
                    LIFT.setPower(0);
                }
            }

            do
            {
                power = pidDrive.performPID(getAngle()); // power will be - on right turn.
                fL.setPower(power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(power);
                if(blockPushed && runtime.milliseconds()>=starttime + 400){
                    CLAW.setPosition(0.03);
                    blockPushed = false;
                }
                if((!touch.getState()||LIFT.getCurrentPosition()<5) && LIFT.getPower()<0 ){
                    LIFT.setPower(0);
                }
            } while (opModeIsActive() && !pidDrive.onTarget());
        }
        else    // left turn.


            do
            {
                pow = pidDrive.performPID(getAngle()); // power will be + on left turn.
                updateT();
                fL.setPower(power-pow);
                fR.setPower(power+pow);
                bL.setPower(power-pow);
                bR.setPower(power+pow);
                if (((2*ticks)-bL.getCurrentPosition()-bR.getCurrentPosition())<100){
                    working = false;
                }
                if(blockPushed && runtime.milliseconds()>=starttime + 400){
                    CLAW.setPosition(0.03);
                    blockPushed = false;
                }
                if((!touch.getState()||LIFT.getCurrentPosition()<5) && LIFT.getPower()<0 ){
                    LIFT.setPower(0);
                }
            } while (opModeIsActive() && runtim2.seconds()<5 && working );

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
        sleep(500);

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
            if(liftgoingup){
            }else if(LIFT.getCurrentPosition()>=50){
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
    private void sleep(int millis){
        runtim2.reset();
        while(opModeIsActive() && runtim2.milliseconds()<millis){
        }
    }
    private void moveWhileUsingLift(int ticks, double power, int position, double liftpower){
        moveWhileUsingLift(ticks, power, position, liftpower, new double[]{0,0,0,0});
    }
    private void moveWhileUsingLift(int ticks, double power, int position, double liftpower, double[] endPowers){
        resetEncoders();
        runtim2.reset();
        boolean targetGreater = position > LIFT.getCurrentPosition();
        if(ticks < 0){
            fL.setPower(-power);
            fR.setPower(-power);
            bL.setPower(-power);
            bR.setPower(-power);
            while(opModeIsActive()&&runtim2.seconds()<4&&averageTicks()>ticks){
                if(LIFT.getPower()<0 && (LIFT.getCurrentPosition()<10 || !touch.getState())){
                    LIFT.setPower(0);
                }else if(targetGreater && LIFT.getCurrentPosition()<position){
                    LIFT.setPower(liftpower);
                }else if(!targetGreater && LIFT.getCurrentPosition()>position){
                    LIFT.setPower(-liftpower);
                }else{
                    if(LIFT.getCurrentPosition()>100)
                        LIFT.setPower(0.2);
                    else
                        LIFT.setPower(0);
                }
                updateT();
            }
        }else{
            fL.setPower(power);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(power);
            while(opModeIsActive()&&runtim2.seconds()<4&&averageTicks()<ticks){
                if(LIFT.getPower()<0 && (LIFT.getCurrentPosition()<10 || !touch.getState())){
                    LIFT.setPower(0);
                }else if(targetGreater && LIFT.getCurrentPosition()<position){
                    LIFT.setPower(liftpower);
                }else if(!targetGreater && LIFT.getCurrentPosition()>position){
                    LIFT.setPower(-liftpower);
                }else{
                    if(LIFT.getCurrentPosition()>100)
                        LIFT.setPower(0.2);
                    else
                        LIFT.setPower(0);
                }
                updateT();
            }
        }
        fL.setPower(endPowers[0]);
        fR.setPower(endPowers[1]);
        bL.setPower(endPowers[2]);
        bR.setPower(endPowers[3]);

    }


}
