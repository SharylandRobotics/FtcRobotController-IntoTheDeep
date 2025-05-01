package org.firstinspires.ftc.team12397.v2.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team12397.v2.RobotHardware;


@TeleOp(name="Field Centric (2)", group="Robot")
@Disabled
public class FieldCentric2 extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    enum states{
        RETRIEVE,
        ROAM,
        SCORE
    }

    enum handle{
        CLOSE() {
            @Override
            public int getIntVal(){
                return 1;
            }
        },
        OPEN() {
            @Override
            public int getIntVal(){
                return 0;
            }
        },
        INTER() {
            @Override
            public int getIntVal(){
                return -1;
            }
        },
        OTHER() {
            @Override
            public int getIntVal(){
                return 2;
            }
        };

        public abstract int getIntVal();
    }

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        double drive = 0;
        double strafe = 0;
        double turn = 0;

        int pinchTimer = 0;
        boolean pinchToggle = false;

        int outPinchTimer = 0;
        boolean outPinchToggle = false;

        handle inClawPinch = handle.OPEN;
        handle outClawPinch = handle.OPEN;

        states currentState = states.ROAM;

        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn  =  gamepad1.right_stick_x;

            robot.driveFieldCentric(drive, strafe, turn);

            if (gamepad2.left_bumper){
                if (pinchTimer >= 4) {
                    pinchToggle = !pinchToggle;
                    pinchTimer = 0;
                }
            }
            if(pinchToggle) {
                inClawPinch = handle.CLOSE;
            }else{
                inClawPinch = handle.OPEN;
            }
            if (gamepad2.right_bumper){
                if (outPinchTimer >= 4) {
                    outPinchToggle = !outPinchToggle;
                    outPinchTimer = 0;
                }
            }
            if(outPinchToggle){
                outClawPinch = handle.CLOSE;
            }else{
                outClawPinch = handle.OPEN;
            }
            robot.setOutClawPinch(outClawPinch.getIntVal());
            robot.setInClawPinch(inClawPinch.getIntVal());

            if (gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0){
                if (currentState == states.RETRIEVE){

                } else if (currentState == states.SCORE){

                }
            }

            if (gamepad2.dpad_up){
                currentState = states.RETRIEVE;
            }

            if (gamepad2.b){
                currentState = states.SCORE;
            }

            pinchTimer++;
            outPinchTimer++;
            sleep(25);
        }
    }
}
