package org.firstinspires.ftc.teamcode.autotest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.Odometry;

import java.util.Arrays;

@Disabled
@Autonomous(name="Autonomous: test")
public class AutonomousTest extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();
    Odometry odometry = new Odometry();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     countsPerMotorRev   = 8192 ;    // eg: TETRIX Motor Encoder
    static final double     wheelDiameterCm     = 6.0 ;     // For figuring circumference
    static final double     countsPerCm         = (countsPerMotorRev / (wheelDiameterCm * Math.PI));

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();
        resetRight();
        resetLeft();
        resetSide();

        double[][] targetPoints= {{0, 0, 0},
                                  {100, 0, 0},
                                  {100, 50, 0},
                                  {100, 50, 90},
                                  {50, 0, 0} }; //array of points we set for the robot as its path
        int targetCounter = 0;//tracks which point in the array is the next target, increased every time a target point is reached by 1

        double[] currentPos= {0,0,0};

        double rPrevPos;
        double rCurrPos = 0;

        double lPrevPos;
        double lCurrPos = 0;

        double sPrevPos;
        double sCurrPos = 0;

        double start, stop;

        while(opModeIsActive()){
            start = System.nanoTime();//only to track speed of odometry update

            rPrevPos = rCurrPos;//calculate distance traveled by each odo wheel
            rCurrPos = robot.encR.getCurrentPosition();
            double dR = (rCurrPos - rPrevPos)/countsPerCm;

            lPrevPos = lCurrPos;
            lCurrPos = robot.encL.getCurrentPosition();
            double dL = (lCurrPos - lPrevPos)/countsPerCm;

            sPrevPos = sCurrPos;
            sCurrPos = robot.encS.getCurrentPosition();
            double dS = (sCurrPos - sPrevPos)/countsPerCm;

            stop = System.nanoTime();

            currentPos = odometry.nowPos(currentPos, dR, dL, dS);//calculate current position based on distance travelled by each odo wheel

            if(odometry.distanceCheck(currentPos[0], currentPos[1], currentPos[2], targetPoints[targetCounter][0], targetPoints[targetCounter][1])){//check if robot reached its target
                targetCounter++;
            }

            double[] power = odometry.calc(targetPoints[targetCounter][0], targetPoints[targetCounter][1], targetPoints[targetCounter][2], currentPos[0],currentPos[1],currentPos[2]);//calculate power needed to give wheels to move towards target

            robot.leftFront.setPower(power[0]);//give power to wheels
            robot.leftBack.setPower(power[1]);
            robot.rightFront.setPower(power[2]);
            robot.rightBack.setPower(power[3]);

            //print some important values into telemetry (display on phone)
            telemetry.addData("power", Arrays.toString(power));
            telemetry.addData("current pos", Arrays.toString(currentPos));
            telemetry.addData("target", Arrays.toString(targetPoints[targetCounter]));
            telemetry.addData("time", (stop - start)/1000000000);
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
    }//reset encoders before start
    private void resetLeft(){
        robot.encL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void resetSide(){
        robot.encS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

}
