package org.firstinspires.ftc.teamcode.drivemodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareRobot;

@TeleOp(name="Basic Drive Mode", group="Linear Opmode")
public class BasicDriveMode extends LinearOpMode {
    HardwareRobot robot = new HardwareRobot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);
        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            double turnCoe = Range.clip(1 - gamepad1.left_trigger, 0.3, 1);
            double speedCoe = Range.clip(1 - gamepad1.left_trigger, 0.3, 1.0);

            double x = gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.update();

            double turn = gamepad1.right_stick_x * turnCoe;

            double iPower = Range.clip((y - x) * speedCoe, -1.0, 1.0);
            double kPower = Range.clip((y + x) * speedCoe, -1.0, 1.0);

            robot.leftFront.setPower(Range.clip(iPower - turn, -1.0, 1.0));
            robot.rightBack.setPower(Range.clip(iPower + turn, -1.0, 1.0));

            robot.rightFront.setPower(Range.clip(kPower + turn, -1.0, 1.0));
            robot.leftBack.setPower(Range.clip(kPower - turn, -1.0, 1.0));

            if (gamepad1.a){
                robot.leftFront.setPower(0);
                robot.rightBack.setPower(0);
                robot.rightFront.setPower(0);
                robot.leftBack.setPower(0);
                break;
            }
        }
    }
}
