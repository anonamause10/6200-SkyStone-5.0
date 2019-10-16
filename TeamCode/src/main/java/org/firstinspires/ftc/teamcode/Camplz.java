/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
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


import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


import java.util.ArrayList;
import java.util.List;

@TeleOp
public class Camplz extends LinearOpMode
{
    OpenCvCamera phoneCam;
    private Mat displayMat = new Mat(); // Display debug info to the screen (this is what is returned)
    private Mat workingMat = new Mat(); // Used for preprocessing and working with (blurring as an example)
    private Mat mask       = new Mat(); // Mask returned by color filter
    private Mat hierarchy  = new Mat(); // hierarchy used by coutnours
    private boolean found    = false; // Is the gold mineral found
    private Point   screenPosition = new Point(); // Screen position of the mineral
    private Rect    foundRect = new Rect(); // Found rect
    private List<Mat> channels = new ArrayList<>();
    private double perfectRatio = 1.67;
    private Size adjustedSize;
    public Size   downscaleResolution = new Size(320, 240);
    public boolean useFixedDownscale = false;
    private Size initSize;
    public double downscale = 0.5;

    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        phoneCam.setPipeline(new SamplePipeline());

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */

        phoneCam.startStreaming(1280, 720);

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                phoneCam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * The "if" statements below will pause the viewport if the "X" button on gamepad1 is pressed,
             * and resume the viewport if the "Y" button on gamepad1 is pressed.
             */
            else if(gamepad1.x)
            {
                phoneCam.pauseViewport();
            }
            else if(gamepad1.y)
            {
                phoneCam.resumeViewport();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */

        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            Log.w("sizes","Input size: "+input.depth());
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
            initSize = input.size();

            if(useFixedDownscale){
                adjustedSize = downscaleResolution;
            }else{
                adjustedSize = new Size(initSize.width * downscale, initSize.height * downscale);
            }
            input.copyTo(displayMat);
            input.copyTo(workingMat);

            Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
            Mat colormat = workingMat.clone();
            channels = new ArrayList<>();

            Imgproc.cvtColor(colormat, colormat, Imgproc.COLOR_RGB2YUV);
            Imgproc.GaussianBlur(colormat,colormat,new Size(3,3),0);
            Core.split(colormat, channels);
            if(channels.size() > 0){
                Imgproc.threshold(channels.get(1), mask, 70, 255, Imgproc.THRESH_BINARY_INV);
            }


            for(int i=0;i<channels.size();i++){
                channels.get(i).release();
            }

            colormat.release();

            List<MatOfPoint> contoursYellow = new ArrayList<>();
            Imgproc.findContours(mask      , contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
                Imgproc.drawContours(displayMat,contoursYellow,-1,new Scalar(230,70,70),2);
            Rect bestRect = null;
            double bestDiffrence = Double.MAX_VALUE; // MAX_VALUE since less diffrence = better
            for(MatOfPoint cont : contoursYellow){
                double score = calculateScore(cont); // Get the diffrence score using the scoring API

                // Get bounding rect of contour
                Rect rect = Imgproc.boundingRect(cont);
                Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0,0,255),2); // Draw rect
                Imgproc.putText(displayMat, ""+(int)(score), rect.tl(),0,0.3,new Scalar(255,255,255));

                // If the result is better then the previously tracked one, set this rect as the new best
                if(score < bestDiffrence){
                    bestDiffrence = score;
                    bestRect = rect;
                }
            }
            if(bestRect != null){
                // Show chosen result
                Imgproc.rectangle(displayMat, bestRect.tl(), bestRect.br(), new Scalar(255,0,0),4);
                Imgproc.putText(displayMat, "Chosen", bestRect.tl(),0,0.1,new Scalar(255,255,255));

                screenPosition = new Point(bestRect.x, bestRect.y);
                foundRect = bestRect;
                found = true;
            }else{
                found = false;
            }


            //Print result
            Imgproc.putText(displayMat,"Result: " + screenPosition.x +"/"+screenPosition.y,new Point(10,getAdjustedSize().height-30),0,1, new Scalar(255,255,0),1);

            Log.w("sizes","Display size: "+displayMat.depth());

            return displayMat;


        }

        public double calculateScore(Mat input){
            double totalScore = 0;

            if(!(input instanceof MatOfPoint)) return Double.MAX_VALUE;
            MatOfPoint contour = (MatOfPoint) input;
            double score = Double.MAX_VALUE;

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(contour);
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            double right = -100*(rect.x+rect.width);

            double cubeRatio = Math.max(Math.abs(h/w), Math.abs(w/h)); // Get the ratio. We use max in case h and w get swapped??? it happens when u account for rotation
            double ratioDiffrence = Math.abs(cubeRatio - perfectRatio);
            double ratioscore = ratioDiffrence*3;

            MatOfPoint contourA = (MatOfPoint) input;
            double area = Imgproc.contourArea(contourA);


            double areascore = -area;
            totalScore += ratioscore;
            totalScore += areascore;
            totalScore+=right;
            return totalScore;

        }

        public Size getAdjustedSize() {
            return adjustedSize;
        }


    }
}
