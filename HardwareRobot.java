package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareRobot {
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;

    public DcMotor encR = null;
    public DcMotor encS = null;
    public DcMotor encL = null;

    public DcMotor lift = null;

    public Servo spinner = null;
    public Servo grabber = null;

    HardwareMap hdwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public void HardwareMap(){

    }

    public void init(HardwareMap ahwMap){
        hdwMap = ahwMap;
        leftFront = hdwMap.get(DcMotor.class, "leftFront");
        rightFront = hdwMap.get(DcMotor.class, "rightFront");
        leftBack = hdwMap.get(DcMotor.class, "leftBack");
        rightBack = hdwMap.get(DcMotor.class, "rightBack");

        encR = hdwMap.get(DcMotor.class, "encr");
        encL = hdwMap.get(DcMotor.class, "encl");
        encS = hdwMap.get(DcMotor.class, "encs");

        lift = hdwMap.get(DcMotor.class, "lift");

        spinner = hdwMap.get(Servo.class, "spinner");
        grabber = hdwMap.get(Servo.class, "grabber");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        encR.setDirection(DcMotor.Direction.REVERSE);
        //remove line below for front-facing chassis
        encS.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
