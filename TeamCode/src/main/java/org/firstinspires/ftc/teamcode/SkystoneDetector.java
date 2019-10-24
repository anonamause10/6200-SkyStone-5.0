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
import com.qualcomm.robotcore.hardware.HardwareMap;

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


public class SkystoneDetector
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
    private double perfectRatio = 0.5;
    private Size adjustedSize;
    public Size   downscaleResolution = new Size(320, 240);
    public boolean useFixedDownscale = false;
    private Size initSize;
    private SamplePipeline pipeline;
    public double downscale = 0.5;
    public boolean right;

    public SkystoneDetector(HardwareMap hardwareMap, boolean webcam, boolean right)
    {
        this.right = right;
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = webcam ? new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId): new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

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
        pipeline = new SamplePipeline();
        phoneCam.setPipeline(pipeline);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an ex\eption
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        /*
         * Wait for the user to press start on the Driver Station
         */

    }

    public double getDist(){
        return pipeline.getDist();
    }

    public double getPos(){
        return pipeline.getPos();
    }

    public void stop(){
        phoneCam.stopStreaming();
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

        private double dist = 320.0;
        private int pos = 2;


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
                Imgproc.putText(displayMat, ""+(int)(score), rect.tl(),0,0.5,new Scalar(255,255,255));

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

            if(bestRect!=null){
                Rect black = skystonepos(workingMat, bestRect);
                pos = 2;
                dist = 320.0;
                if(right) { //if we're looking from the right:
                    if (black != null) {
                        dist = Math.abs(black.br().x - bestRect.br().x);
                        pos = (int) Math.round((dist) / 160.0);
                    }
                }
                else{ //if we're looking from the left:
                    if(black!=null) {
                        dist = Math.abs(bestRect.tl().x - black.tl().x);
                        pos = (int) Math.round((dist) / 160.0);
                    }
                }
            }


            return displayMat;
        }

        public double getDist() {
            return dist;
        }

        public int getPos() {
            return pos;
        }

        public Rect skystonepos(Mat input, Rect rect){
            int pos = 0;
            Mat newMask = new Mat();
            int height = rect.height;
            Rect sub = null;
            if(right){
                Point tr = new Point(rect.br().x,rect.tl().y);
                sub = new Rect(0,(int)(tr.y),(int)(tr.x),rect.height);
            }
            else{
                Log.w("Tiggle", rect.toString());
                sub = new Rect((int)(rect.x),(int)(rect.y),input.width() - (int)(rect.x),rect.height);
            }
            Imgproc.rectangle(displayMat, sub, new Scalar(0, 255, 0), 4);
            Mat submat = input.submat(sub);

            Imgproc.GaussianBlur(submat,submat,new Size(5,5),0);
            Mat colormat = submat.clone();
            channels = new ArrayList<>();

            Imgproc.cvtColor(colormat, colormat, Imgproc.COLOR_RGB2GRAY);
            Imgproc.GaussianBlur(colormat,colormat,new Size(3,3),0);
            Core.split(colormat, channels);
            if(channels.size() > 0){
                Imgproc.threshold(channels.get(0), newMask, 25, 255, Imgproc.THRESH_BINARY_INV);
            }
            for(int i=0;i<channels.size();i++) {
                channels.get(i).release();
            }

            colormat.release();
            List<MatOfPoint> contoursBlack = new ArrayList<>();
            Imgproc.findContours(newMask, contoursBlack, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(displayMat,contoursBlack,-1, new Scalar(255,255,255), 2, 1, hierarchy,1,new Point(right?0:rect.x,rect.y));
            Rect bestblack = null;
            double bestDiffblack = Double.MAX_VALUE; // MAX_VALUE since less diffrence = better
            for(MatOfPoint cont : contoursBlack){
                double score = calculateScore(cont); // Get the diffrence score using the scoring API

                // Get bounding rect of contour
                Rect recto = Imgproc.boundingRect(cont);

                // If the result is better then the previously tracked one, set this rect as the new best
                if(score < bestDiffblack){
                    bestDiffblack = score;
                    bestblack = recto;
                }
            }

            if(bestblack != null) {
                // Show chosen result
                Rect drawblack = new Rect((int)bestblack.tl().x+(right?0:rect.x), (int)bestblack.tl().y+rect.y,bestblack.width, bestblack.height);
                Imgproc.rectangle(displayMat, drawblack, new Scalar(255, 255, 0), 4);

                bestblack = drawblack;

            }
            return bestblack;

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
            double rightDist = -225*(right?(rect.x+rect.width):(workingMat.width()-rect.x));

            double cubeRatio = Math.max(Math.abs(h/w), Math.abs(w/h)); // Get the ratio. We use max in case h and w get swapped??? it happens when u account for rotation
            double ratioDiffrence = Math.abs(cubeRatio - perfectRatio);
            double ratioscore = ratioDiffrence*3;

            MatOfPoint contourA = (MatOfPoint) input;
            double area = Imgproc.contourArea(contourA);


            double areascore = -area;
            totalScore += ratioscore;
            totalScore += areascore;
            totalScore+=rightDist;
            return totalScore  ;

        }
        public Size getAdjustedSize() {
            return adjustedSize;
        }


    }
}
