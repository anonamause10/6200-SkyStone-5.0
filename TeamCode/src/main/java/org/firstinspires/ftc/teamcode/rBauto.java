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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


@Autonomous(name="RED STONE Auto", group="ree")

public class rBauto extends LinearOpMode
{
    private double FINPERTICK = 1000/23.5;
    private double SINPERTICK = 1000/16.5;
    private double DEGREESPERTICK = 1000/90;
    private double STRAFEERROR = 3/23.5;

    private int skyStonPos = 0;
    private int[] skyStonesPos = new int[6];
    private double[] vuXYZ = new double[3];
    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private static final String VUFORIA_KEY =
            "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    double objTurn = 0;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor fL = null;
    private DcMotor fR = null;
    private DcMotor bL = null;
    private DcMotor bR = null;
    private DcMotor uD = null;
    private Servo servo =null;

    private double voltage = 0.0;
    private double scale = 0.0;

    private int downPosition = 300;
    private int incremented = 0;
    private int increment = 50;


    @Override
    public void runOpMode() {
        webcamName = hardwareMap.get(WebcamName.class, "cam");


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
        parameters.fillCameraMonitorViewParent = true;

        parameters.cameraName = webcamName;
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
        uD = hardwareMap.get(DcMotor.class, "uD");
        uD.setDirection(DcMotor.Direction.FORWARD);
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

        //START AUTO HERE LMAO

        moveFB(-18);
        boolean targetFound = false;
        if(targetFound){

        }else{
            strafe(5);
            servoToBlock();
            moveFB(12);
            servoUp();
            strafe(12);
            eTurn(90);
            strafe(-9);
            moveFB(48);
        }


    }

    private void eTurn(double degrees){ //COUNTER CLOCKWISE IS POSITIVE
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
        while(fL.isBusy() && runtime.seconds()<28) {
            if (Math.abs(fL.getCurrentPosition() - fL.getTargetPosition()) < 3) {
                fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }
    private void moveFB(double inches){
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
        while(fL.isBusy() && runtime.seconds()<28) {
            if (Math.abs(fL.getCurrentPosition() - fL.getTargetPosition()) < 3) {
                fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }
    private void strafe(double inches){
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
        while(fL.isBusy() && runtime.seconds()<28) {
            if (Math.abs(fL.getCurrentPosition() - fL.getTargetPosition()) < 3) {
                fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        eTurn(inches*STRAFEERROR);
    }
    private void servoToBlock(){
        servo.setPosition(0.5);
    }
    private void servoUp(){
        servo.setPosition(1);
    }
    private void servoToFoun(){
        servo.setPosition(0.35);
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

     boolean runNavig(){
             /*
              * Retrieve the camera we are to use.
              */
             webcamName = hardwareMap.get(WebcamName.class, "cam");

             /*
              * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
              * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
              * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
              */
             int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
             VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

             // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

             parameters.vuforiaLicenseKey = VUFORIA_KEY;

             /**
              * We also indicate which camera on the RC we wish to use.
              */
             parameters.cameraName = webcamName;

             //  Instantiate the Vuforia engine
             vuforia = ClassFactory.getInstance().createVuforia(parameters);

             // Load the data sets for the trackable objects. These particular data
             // sets are stored in the 'assets' part of our application.
             VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

             VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
             stoneTarget.setName("Stone Target");
             VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
             blueRearBridge.setName("Blue Rear Bridge");
             VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
             redRearBridge.setName("Red Rear Bridge");
             VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
             redFrontBridge.setName("Red Front Bridge");
             VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
             blueFrontBridge.setName("Blue Front Bridge");
             VuforiaTrackable red1 = targetsSkyStone.get(5);
             red1.setName("Red Perimeter 1");
             VuforiaTrackable red2 = targetsSkyStone.get(6);
             red2.setName("Red Perimeter 2");
             VuforiaTrackable front1 = targetsSkyStone.get(7);
             front1.setName("Front Perimeter 1");
             VuforiaTrackable front2 = targetsSkyStone.get(8);
             front2.setName("Front Perimeter 2");
             VuforiaTrackable blue1 = targetsSkyStone.get(9);
             blue1.setName("Blue Perimeter 1");
             VuforiaTrackable blue2 = targetsSkyStone.get(10);
             blue2.setName("Blue Perimeter 2");
             VuforiaTrackable rear1 = targetsSkyStone.get(11);
             rear1.setName("Rear Perimeter 1");
             VuforiaTrackable rear2 = targetsSkyStone.get(12);
             rear2.setName("Rear Perimeter 2");

             // For convenience, gather together all the trackable objects in one easily-iterable collection */
             List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
             allTrackables.addAll(targetsSkyStone);

             /**
              * In order for localization to work, we need to tell the system where each target is on the field, and
              * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
              * Transformation matrices are a central, important concept in the math here involved in localization.
              * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
              * for detailed information. Commonly, you'll encounter transformation matrices as instances
              * of the {@link OpenGLMatrix} class.
              *
              * If you are standing in the Red Alliance Station looking towards the center of the field,
              *     - The X axis runs from your left to the right. (positive from the center to the right)
              *     - The Y axis runs from the Red Alliance Station towards the other side of the field
              *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
              *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
              *
              * Before being transformed, each target image is conceptually located at the origin of the field's
              *  coordinate system (the center of the field), facing up.
              */

             // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
             // Rotated it to to face forward, and raised it to sit on the ground correctly.
             // This can be used for generic target-centric approach algorithms
             stoneTarget.setLocation(OpenGLMatrix
                     .translation(0, 0, stoneZ)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

             //Set the position of the bridge support targets with relation to origin (center of field)
             blueFrontBridge.setLocation(OpenGLMatrix
                     .translation(-bridgeX, bridgeY, bridgeZ)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

             blueRearBridge.setLocation(OpenGLMatrix
                     .translation(-bridgeX, bridgeY, bridgeZ)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

             redFrontBridge.setLocation(OpenGLMatrix
                     .translation(-bridgeX, -bridgeY, bridgeZ)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

             redRearBridge.setLocation(OpenGLMatrix
                     .translation(bridgeX, -bridgeY, bridgeZ)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

             //Set the position of the perimeter targets with relation to origin (center of field)
             red1.setLocation(OpenGLMatrix
                     .translation(quadField, -halfField, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

             red2.setLocation(OpenGLMatrix
                     .translation(-quadField, -halfField, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

             front1.setLocation(OpenGLMatrix
                     .translation(-halfField, -quadField, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

             front2.setLocation(OpenGLMatrix
                     .translation(-halfField, quadField, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

             blue1.setLocation(OpenGLMatrix
                     .translation(-quadField, halfField, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

             blue2.setLocation(OpenGLMatrix
                     .translation(quadField, halfField, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

             rear1.setLocation(OpenGLMatrix
                     .translation(halfField, quadField, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

             rear2.setLocation(OpenGLMatrix
                     .translation(halfField, -quadField, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

             //
             // Create a transformation matrix describing where the phone is on the robot.
             //
             // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
             // Lock it into Portrait for these numbers to work.
             //
             // Info:  The coordinate frame for the robot looks the same as the field.
             // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
             // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
             //
             // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
             // pointing to the LEFT side of the Robot.
             // The two examples below assume that the camera is facing forward out the front of the robot.

             // We need to rotate the camera around it's long axis to bring the correct camera forward.
             if (CAMERA_CHOICE == BACK) {
                 phoneYRotate = -90;
             } else {
                 phoneYRotate = 90;
             }

             // Rotate the phone vertical about the X axis if it's in portrait mode
             if (PHONE_IS_PORTRAIT) {
                 phoneXRotate = 90 ;
             }

             // Next, translate the camera lens to where it is on the robot.
             // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
             final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
             final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
             final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

             OpenGLMatrix robotFromCamera = OpenGLMatrix
                     .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

             /**  Let all the trackable listeners know where the phone is.  */
             for (VuforiaTrackable trackable : allTrackables) {
                 ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
             }

             // WARNING:
             // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
             // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
             // CONSEQUENTLY do not put any driving commands in this loop.
             // To restore the normal opmode structure, just un-comment the following line:

             // waitForStart();

             // Note: To use the remote camera preview:
             // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
             // Tap the preview window to receive a fresh image.

             targetsSkyStone.activate();

                 // check all the trackable targets to see which one (if any) is visible.
                 targetVisible = false;
                 for (VuforiaTrackable trackable : allTrackables) {
                     if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                         telemetry.addData("Visible Target", trackable.getName());
                         targetVisible = true;

                         // getUpdatedRobotLocation() will return null if no new information is available since
                         // the last time that call was made, or if the trackable is not currently visible.
                         OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                         if (robotLocationTransform != null) {
                             lastLocation = robotLocationTransform;
                         }
                         break;
                     }
                 }

                 // Provide feedback as to where the robot is located (if we know).
                 if (targetVisible) {
                     // express position (translation) of robot in inches.
                     VectorF translation = lastLocation.getTranslation();
                     telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                             translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                     for(int i = 0; i<3; i++){
                         vuXYZ[i] = translation.get(i)/mmPerInch;
                     }
                     // express the rotation of the robot in degrees.
                     Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                     telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                 }
                 else {
                     telemetry.addData("Visible Target", "none");
                 }
                 telemetry.update();

             // Disable Tracking when we are done;
             targetsSkyStone.deactivate();
             return targetVisible;
         }

}
