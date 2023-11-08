package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Objects;


//USE THIS FOR CONE DETECTION


/*
 * Copyright (c) 2020 OpenFTC Team
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



/*
 * This version of the internal camera example uses EasyOpenCV's interface to the
 * Android Camera2 API
 */

@Autonomous
public class ConeDetectionTest2 extends LinearOpMode {
    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        phoneCam.setPipeline(new ConeDetect());

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
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
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addLine("Camera could not be accessed");
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive()) {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("r", ConeDetect.r());
            telemetry.addData("g", ConeDetect.g());
            telemetry.addData("b", ConeDetect.b());
            telemetry.addData("counter", ConeDetect.counter());
            telemetry.addData("cone colour", ConeDetect.analyze());
            telemetry.addData("row", ConeDetect.row());
            telemetry.addData("column", ConeDetect.column());

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

    }

    static class ConeDetect extends OpenCvPipeline {

        static int colour; //0=red, 1=green, 2=blue

        Point topRightAnchor = new Point(190, 235);//x: 190, y: 235   ;change these x, y values to change area where colour is measured (image is 320x240 px)
        Point botLeftAnchor = new Point(120, 200); //x: 120, y: 200

        Mat region;
        //Mat Hsv = new Mat();
        //Mat Hue = new Mat();
        Scalar avg;
        boolean end, detect = false;

        static double r;
        static double g;
        static double b;
        static double prevR;
        static double prevG;
        static double prevB;
        static double dR, dG, dB;
        static int counter, row = 0;
        static int i, j;

        /*void inputToHsv(Mat input) {
            Imgproc.cvtColor(input, Hsv, Imgproc.COLOR_BGR2HSV);
            //Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame){
            inputToHsv(firstFrame);
        }*/

        @Override
        public Mat processFrame(Mat input) {
            //inputToHsv(input);

            if (!input.empty()) {
                for (i = input.height()-1; i >= 0; i--) { //i is row (y coord), j is column (x coord)
                    for (j = input.width()-1; j >= 0; j--) {
                        prevR = r;
                        r = input.get(i, j)[0];
                        prevG = g;
                        g = input.get(i, j)[1];
                        prevB = b;
                        b = input.get(i, j)[2];
                        dR = prevR - r;
                        dG = prevG - g;
                        dB = prevB - b;
                        double avg = (r + g + b) / 3.0;
                        if ((counter == 0 && dG + dB - dR > 90) || (counter > 0 && r > avg * 1.2)) {
                            counter++;
                        } else if (counter > 30) {
                            row++;
                            detect = true;
                            if(row == 2){
                                botLeftAnchor = new Point(i, Range.clip(Math.floor(j - 3.0 * counter / 4.0), 0, input.height()-1));
                                topRightAnchor = new Point(Range.clip(Math.floor(i + 4.5 * counter / 4.0), 0, input.width()-1), Range.clip(Math.floor(j - counter / 4.0), 0, input.height()-1));
                                end = true;
                            }
                        }
                        Imgproc.rectangle(input, topRightAnchor, botLeftAnchor, new Scalar(255, 255, 255), 2);
                        if (end) {
                            break;
                        }
                    }
                    counter = 0;
                    if (end) {
                        end = false;
                        break;
                    }
                    if(!detect){
                        row = 0;
                    }
                }

                region = input.submat(new Rect(topRightAnchor, botLeftAnchor));
                avg = Core.mean(region);

                if (avg.val[0] > (avg.val[0] + avg.val[1] + avg.val[2]) / 3 * 1.1) {//val[0] = red, 1 = green, 2 = blue
                    colour = 0;
                    Imgproc.rectangle(input, topRightAnchor, botLeftAnchor, avg, -1);//fill rectangle where avg colour is measured by the avg colour
                } else if (avg.val[1] > (avg.val[0] + avg.val[1] + avg.val[2]) / 3 * 1.1) {
                    colour = 1;
                    Imgproc.rectangle(input, topRightAnchor, botLeftAnchor, avg, -1);
                } else if (avg.val[2] > (avg.val[0] + avg.val[1] + avg.val[2]) / 3 * 1.1) {
                    colour = 2;
                    Imgproc.rectangle(input, topRightAnchor, botLeftAnchor, avg, -1);
                }

                return input;
            }
            return input;
        }

        public static int analyze(){
            return colour;
        }

        public static double r(){
            return r;
        }

        public static double g(){
            return g;
        }

        public static double b(){
            return b;
        }

        public static int counter(){
            return counter;
        }

        public static int row(){
            return i;
        }

        public static int column(){
            return j;
        }
    }
}