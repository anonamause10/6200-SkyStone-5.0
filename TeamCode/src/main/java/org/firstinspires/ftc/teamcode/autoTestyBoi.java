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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.Pif;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


@TeleOp(name="Auto big rart", group="ree")

public class autoTestyBoi extends LinearOpMode
{
    private double FINPERTICK = 1000/23.5;
    private double SINPERTICK = 1000/19.5;
    private double DEGREESPERTICK = 1000/85;
    private double STRAFEERROR = 3/19.5;

    //WebcamName webcamName = null;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor fL = null;
    private DcMotor fR = null;
    private DcMotor bL = null;
    private DcMotor bR = null;
    private DcMotor IN1 = null;
    private DcMotor IN2 = null;
    private Servo servo =null;

    private double voltage = 0.0;
    private double scale = 0.0;

    private int downPosition = 300;
    private int incremented = 0;
    private int increment = 50;




    @Override
    public void runOpMode() {
        //SkystoneDetector sky = new SkystoneDetector(hardwareMap, true);
        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
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
        IN1 = hardwareMap.get(DcMotor.class, "IN1");
        IN1.setDirection(DcMotor.Direction.FORWARD);
        IN2 = hardwareMap.get(DcMotor.class,"IN2");
        IN2.setDirection(DcMotor.Direction.FORWARD);
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(1);

        telemetry.addData("Robot", "Initialized");
        voltage = getBatteryVoltage();
        scale = 12.7 / voltage;
        telemetry.addData("Voltage:", voltage);
        telemetry.addData("Scale", scale);
        telemetry.update();
        waitForStart();
        runtime.reset();
        int x = 0;
        boolean aPrev = false;
        boolean bPrev = false;
        boolean dUpPrev = false;
        boolean dDownPrev = false;
        boolean dLeftPrev = false;
        boolean dRightPrev = false;

        while(opModeIsActive()){
            if(gamepad1.a&&!aPrev){
                x++;
            }
            if(gamepad1.b&&!bPrev){
                x--;
            }
            if(gamepad1.dpad_left&&!dLeftPrev){
                eTurn(x);
            }
            if(gamepad1.dpad_right&&!dRightPrev){
                strafe(x);
            }
            if(gamepad1.dpad_down&&!dDownPrev){
            }

            if(gamepad1.left_bumper){
                servoToBlock();
            }
            if(gamepad1.right_bumper){
                servoUp();
            }
            if(gamepad1.dpad_up&&!dUpPrev){
                moveFB(x);
            }

            aPrev = gamepad1.a;
            bPrev = gamepad1.b;
            dDownPrev = gamepad1.dpad_down;
            dUpPrev = gamepad1.dpad_up;
            dLeftPrev = gamepad1.dpad_left;
            dRightPrev = gamepad1.dpad_right;
            telemetry.addData("x", x);
            telemetry.update();
        }


    }

    private void eTurn(double degrees){ //COUNTER CLOCKWISE IS POSITIVE
        runtime.reset();
        int target = (int)(degrees * DEGREESPERTICK);
        fL.setTargetPosition(-target);
        fR.setTargetPosition(target);
        bL.setTargetPosition(-target);
        bR.setTargetPosition(target);
        fL.setPower(0.3);
        fR.setPower(0.3);
        bL.setPower(0.3);
        bR.setPower(0.3);
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean working = true;
        while(fL.isBusy() && runtime.seconds()<5 && working) {
            updateT();
            if (Math.abs(fL.getCurrentPosition() - fL.getTargetPosition()) < 3) {
                fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        try {
            wait(100);
        }catch(Exception E){
        }
    }
    private void moveFB(double inches){
        runtime.reset();
        int target = (int)(inches * FINPERTICK);
        fL.setTargetPosition(target);
        fR.setTargetPosition(target);
        bL.setTargetPosition(target);
        bR.setTargetPosition(target);
        fL.setPower(0.3);
        fR.setPower(0.3);
        bL.setPower(0.3);
        bR.setPower(0.3);
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean working = true;
        while(fL.isBusy() && runtime.seconds()<5 && working) {
            updateT();

            if (Math.abs(fL.getCurrentPosition() - fL.getTargetPosition()) < 3) {
                fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        try {
            wait(100);
        }catch(Exception E){
        }
    }
    private void strafe(double inches){
        runtime.reset();
        int target = (int)(inches * SINPERTICK);
        fL.setTargetPosition(target);
        fR.setTargetPosition(-target);
        bL.setTargetPosition(-target);
        bR.setTargetPosition(target);
        fL.setPower(0.3);
        fR.setPower(0.3);
        bL.setPower(0.3);
        bR.setPower(0.3);
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean working = true;
        while(fL.isBusy() && runtime.seconds()<5&&working) {
            updateT();
            if (Math.abs(fL.getCurrentPosition() - fL.getTargetPosition()) < 3) {
                fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        eTurn(inches*STRAFEERROR);
    }
    private void servoToBlock(){
        servo.setPosition(0.5);
        try {
            wait(100);
        }catch(Exception E){
        }
    }
    private void servoUp(){
        servo.setPosition(1);
        try {
            wait(100);
        }catch(Exception E){
        }
    }
    private void servoToFoun(){
        servo.setPosition(0.35);
        try {
            wait(100);
        }catch(Exception E){
        }
    }
    private void intake(){
        IN1.setPower(0.7);
        IN2.setPower(0.7);
        try {
            wait(100);
        }catch(Exception E){
        }
    }
    private void outtake(){
        IN1.setPower(-0.4);
        IN2.setPower(-0.4);
        try {
            wait(100);
        }catch(Exception E){
        }
    }
    private void intakeOff(){
        IN1.setPower(0);
        IN2.setPower(0);
        try {
            wait(100);
        }catch(Exception E){
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

    private int runDetect( SkystoneDetector sky){
        int result = (int) sky.getPos();
        sky.stop();
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
        telemetry.addData("ServoPos", servo.getPosition());
        telemetry.addData("INTAKE POWER", IN1.getPower());
        telemetry.update();
    }

}

