package org.firstinspires.ftc.teamcode.autotest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ConeDetectionTest2;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.Odometry;

import java.util.Arrays;

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

@Autonomous(name="Autonomous: camera detection test")
public class AutonomousCamDetTest extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();
    Odometry odometry = new Odometry();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     countsPerMotorRev   = 8192 ;    // eg: TETRIX Motor Encoder
    static final double     wheelDiameterCm     = 6.0 ;     // For figuring circumference
    static final double     countsPerCm         = (countsPerMotorRev / (wheelDiameterCm * Math.PI));

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() {

        //init camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.setPipeline(new ConeDetect());

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera could not be accessed");
                telemetry.update();
            }
        });

        robot.init(hardwareMap);

        double[][] targetPoints = new double[5][]; //array of points we set for the robot as its path

        int targetCounter = 0;

        double[] currentPos= {0,0,0};

        double rPrevPos;
        double rCurrPos = 0;

        double lPrevPos;
        double lCurrPos = 0;

        double sPrevPos;
        double sCurrPos = 0;

        double startTime, stopTime;

        boolean scan = true;

        int r = 0;
        int g = 0;
        int b = 0;

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
        resetRight();
        resetLeft();
        resetSide();

        while(opModeIsActive()){
            if(scan){//scan avg colour of cone 5 times at the beginning of auto
                for (int i = 0; i < 5; i++){
                    double[] result = ConeDetect.analyze();
                    double avg = result[0] + result [1] + result[2] / 3 * 1.1;
                    if (result[0] > avg) {//add +1 to r, g or b depending on detected avg colour
                        r++;
                    } else if (result[1] > avg) {
                        g++;
                    } else if (result[2] > avg) {
                        b++;
                    }
                }
                if(r > g && r > b){ // if red, go forward
                    targetPoints = new double[][]{{0, 0, 0},
                            {100, 0, 0},
                            {100, 50, 0},
                            {100, 50, 90},
                            {50, 0, 0}};
                } else if(g > b){ // if green, go right
                    targetPoints = new double[][]{{0, 0, 0},
                            {0, 100, 0},
                            {-50, 100, 0},
                            {-50, 100, 90},
                            {0, 50, 0}};
                } else if(b > g){ //if blue, go back
                    targetPoints = new double[][]{{0, 0, 0},
                            {-100, 0, 0},
                            {-100, -50, 0},
                            {-100, -50, -90},
                            {-50, 0, 0}};
                }
                scan = false;
            }

            startTime = System.nanoTime();

            rPrevPos = rCurrPos;
            rCurrPos = robot.encR.getCurrentPosition();
            double dR = (rCurrPos - rPrevPos)/countsPerCm;

            lPrevPos = lCurrPos;
            lCurrPos = robot.encL.getCurrentPosition();
            double dL = (lCurrPos - lPrevPos)/countsPerCm;

            sPrevPos = sCurrPos;
            sCurrPos = robot.encS.getCurrentPosition();
            double dS = (sCurrPos - sPrevPos)/countsPerCm;

            stopTime = System.nanoTime();

            currentPos = odometry.nowPos(currentPos, dR, dL, dS);

            if(odometry.distanceCheck(currentPos[0], currentPos[1], currentPos[2], targetPoints[targetCounter][0], targetPoints[targetCounter][1])){
                targetCounter++;
            }

            double[] power = odometry.calc(targetPoints[targetCounter][0], targetPoints[targetCounter][1], targetPoints[targetCounter][2], currentPos[0],currentPos[1],currentPos[2]);

            robot.leftFront.setPower(power[0]);
            robot.leftBack.setPower(power[1]);
            robot.rightFront.setPower(power[2]);
            robot.rightBack.setPower(power[3]);

            telemetry.addData("power", Arrays.toString(power));
            telemetry.addData("current pos", Arrays.toString(currentPos));
            telemetry.addData("target", Arrays.toString(targetPoints[targetCounter]));
            telemetry.addData("time", (stopTime - startTime)/1000000000);
            telemetry.addData("left", lCurrPos/countsPerCm);
            telemetry.addData("right", rCurrPos/countsPerCm);
            telemetry.addData("side", sCurrPos/countsPerCm);
            telemetry.addData("dL", dL);
            telemetry.addData("dR", dR);
            telemetry.addData("dS", dS);
            telemetry.update();
        }
    }

    private void resetRight(){
        robot.encR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void resetLeft(){
        robot.encL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void resetSide(){
        robot.encS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }


    static class ConeDetect extends OpenCvPipeline {

        static int colour; //0=red, 1=green, 2=blue

        Point topRightAnchor = new Point(190, 235);
        Point botLeftAnchor = new Point(120, 200);

        Mat region;
        //Mat YCrCb = new Mat();
        //Mat Cb = new Mat();
        static Scalar avg;
        boolean end = false;

        static double r;
        static double g;
        static double b;
        static int counter;
        static int i;
        static int j;

        /*void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }*/

        /*@Override
        public void init(Mat firstFrame){
            inputToCb(firstFrame);
        }*/

        @Override
        public Mat processFrame(Mat input) {
            //inputToCb(input);

            /*r = input.get(input.height()/2, input.width()/2)[0];
            g = input.get(input.height()/2, input.width()/2)[1];
            b = input.get(input.height()/2, input.width()/2)[2];*/

            if (!input.empty()) {
                /*for (i = 1; i < input.height(); i++) { //i is row (y coord), j is column (x coord)
                    for (j = 1; j < input.width(); j++) {
                        r = input.get(i, j)[0];
                        g = input.get(i, j)[1];
                        b = input.get(i, j)[2];
                        double avg = (r + g + b) / 3;
                        if (r > avg * 1.5) {
                            counter++;
                        } else if (counter > 40) {
                            botLeftAnchor = new Point(i, Range.clip(Math.floor(j - 3.0 * counter / 4.0), 0, input.height()));
                            topRightAnchor = new Point(Range.clip(Math.floor(i + 4.5 * counter / 4.0), 0, input.width()), Range.clip(Math.floor(j - counter / 4.0), 0, input.height()));
                            end = true;
                        }
                        Imgproc.rectangle(input, topRightAnchor, botLeftAnchor, new Scalar(0, 0, 255), 2);
                        if (end) {
                            break;
                        }
                    }
                    counter = 0;
                    if (end) {
                        end = false;
                        break;
                    }
                }*/

                region = input.submat(new Rect(topRightAnchor, botLeftAnchor));
                avg = Core.mean(region);

                if (avg.val[0] > (avg.val[0] + avg.val[1] + avg.val[2]) / 3 * 1.1) {
                    colour = 0;
                    Imgproc.rectangle(input, topRightAnchor, botLeftAnchor, avg, -1);
                } else if (avg.val[1] > (avg.val[0] + avg.val[1] + avg.val[2]) / 3 * 1.1) {
                    colour = 1;
                    Imgproc.rectangle(input, topRightAnchor, botLeftAnchor, avg, -1);
                } else if (avg.val[3] > (avg.val[0] + avg.val[1] + avg.val[2]) / 3 * 1.1) {
                    colour = 2;
                    Imgproc.rectangle(input, topRightAnchor, botLeftAnchor, avg, -1);
                }

                return input;
            }
            return null;
        }

        public static double[] analyze(){
            return new double[]{avg.val[0], avg.val[1], avg.val[2]};
        }
        /*
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
        }*/
    }
}

