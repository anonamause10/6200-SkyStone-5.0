/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the fofllowing conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;


import android.graphics.ImageFormat;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.internal.camera.CameraManagerInternal;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
public class TestDetection extends LinearOpMode
{
    private SkystoneDetectorNew detector;
    int x = 40;
    int y = 40;
    boolean leftPrev = false;
    boolean rightPrev = false;
    boolean upPrev = false;
    boolean downPrev = false;


    @Override
    public void runOpMode()
    {
        boolean blockPos = true;
        /*
         * Wait for the user to press start on the Driver Station
         */
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.addData("blockpos", blockPos);
            if(gamepad1.x){
                blockPos = false;
            }else if(gamepad1.a){
                blockPos = true;
            }
            telemetry.update();
        }
        detector = new SkystoneDetectorNew(hardwareMap, true, blockPos,true);

        while (opModeIsActive())
        {

            /*
             * Send some stats to the telemetry
             */


            telemetry.addData("Distance: ", detector.getDist());
            telemetry.addData("Position: ", detector.getPos());
            telemetry.addData("x: ", detector.getX());
            telemetry.addData("y: ", detector.getY());
            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.dpad_up&&!upPrev){
                y+=5;
            }
            if(gamepad1.dpad_down&&!downPrev){
                y-=5;
            }
            if(gamepad1.dpad_left&&!leftPrev){
                x-=5;
            }
            if(gamepad1.dpad_right&&!rightPrev){
                x+=5;
            }
            detector.setPos(x,y);
            if(gamepad1.a)
            {

                detector.stop();
                //webcam.closeCameraDevice();
            }
            upPrev = gamepad1.dpad_up;
            downPrev = gamepad1.dpad_down;
            leftPrev = gamepad1.dpad_left;
            rightPrev = gamepad1.dpad_right;

        }
    }


}
