package org.firstinspires.ftc.teamcode.drivemodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareRobot;

/**ADB guide:
 *
 * 1) pripojit mobil a pc na stejnou wifi a spojit je pres usb
 * 2) v cmd odnavigovat do lokace Android SDK (na liste rozkliknout settings(vpravo nahore) a das SDK Manager
 * 3) ve slozce Sdk jit do slozky platform-tools (cd platform-tools)
 * 4) adb tcpip 5555
 * 5) adb connect *IP adresa mobilu (jde najit v nastaveni wifi na mobilu)*:5555
 * 5.5) adb devices -> pokud to napise ip adresu mobilu jako pripojeny device tak se podarilo mobil a pc spojit
 * 6) odpojit mobil od pocitace pres usb - android studio by vedle tlacitka run melo porad ukazovat mobil jako pripojeny device i kdyz uz neni pripojeny kabelem
 * 7) hotovo - ted muzete buildovat bez kabelu
 *
 */

@TeleOp(name="Test Drive Mode", group="Linear Opmode")
public class TestDriveMode extends LinearOpMode {
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
            if (gamepad1.left_stick_y >= 0){
                robot.leftFront.setPower(gamepad1.left_stick_y);
                telemetry.addData("motor power", gamepad1.left_stick_y);
                telemetry.update();
            }
        }
    }
}
