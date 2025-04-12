package org.firstinspires.ftc.team12397.v2.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.team12397.v2.RobotHardware;

@TeleOp(name="FieldCentric", group="Robot")

public class FieldCentric extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        double rotation = 0;

        Gamepad luisL = gamepad1;
        Gamepad alexH = gamepad2;



        while (opModeIsActive()) {

            drive = luisL.left_stick_y;
            strafe = luisL.left_stick_x;
            turn = luisL.right_stick_x;


            robot.driveFieldCentric(drive, strafe, turn);

            if(alexH.y){
                rotation = robot.ROTATION_START;
            } else if (alexH.a){
                rotation = robot.ROTATION_90;
            }

            robot.RotateSlides(rotation);

        }
    }

}
