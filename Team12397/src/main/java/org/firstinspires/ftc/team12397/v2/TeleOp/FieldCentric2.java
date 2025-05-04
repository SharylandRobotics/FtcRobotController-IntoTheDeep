package org.firstinspires.ftc.team12397.v2.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.team12397.v2.RobotHardware;


@TeleOp(name="Field Centric (2)", group="Robot")
public class FieldCentric2 extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    private final double OUTTAKE_MAX = 0.8;
    private final double OUTTAKE_MIN = 0.32;
    private final double OUTTAKE_MID = 0.8;

    private final double EXTEND_MAX = 1;
    private final double EXTEND_MID = 0.4;
    private final double EXTEND_MIN = 0;

    private final double PITCH_MAX = 1;
    private final double PITCH_MID = 0.65;
    private final double PITCH_MIN = 0.15;

    private final double IN_YAW_MAX = 0.4225;
    private final double IN_YAW_MIN = 0;

    private final double OUT_YAW_MAX = 0.7;
    private final double OUT_YAW_MIN = 0.04;

    private final double SLIDE_RUNG = 1.5;

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

        double inClawYaw = 0;
        double outTakePos = 0.55;
        double extendPos = 0;
        double pitchPos = 0.5;
        double outClawYaw = 0;

        int pinchTimer = 0;
        boolean pinchToggle = false;

        int outPinchTimer = 0;
        boolean outPinchToggle = false;

        double YawTimer = 0;
        double PitchTimer = 0;
        double dpadUpTimer = 0;
        double bTimer = 0;

        double slide = 0;
        double slideFudge = 0;

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
            if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0){
                slideFudge += 0.05 * gamepad1.left_trigger;
                slideFudge = Math.min(1.5, slideFudge);

                slideFudge -= 0.05 * gamepad1.right_trigger;
                slideFudge = Math.max(-1.5, slideFudge);
            }


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


            // triggers move yaw if RETRIEVE, left one moves outTake back in if SCORE
            if (gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0){

                if (currentState == states.RETRIEVE || currentState == states.ROAM){
                    inClawYaw += 0.115 * gamepad2.left_trigger;
                    inClawYaw = Math.min(1, inClawYaw);

                    inClawYaw -= 0.115 * gamepad2.right_trigger;
                    inClawYaw = Math.max(0, inClawYaw);

                } else if (currentState == states.SCORE){
                    outTakePos -= 0.01 * gamepad2.left_trigger;
                    outTakePos = Math.min(OUTTAKE_MID, outTakePos);
                }
                if (pitchPos < 0.65){
                    inClawYaw = 0;
                }
            }


            // left stick extends extender in RETRIEVE,
            if (gamepad2.left_stick_y != 0){
                if (currentState == states.RETRIEVE){
                    extendPos = Range.clip(extendPos, 0, 1);
                    extendPos += -gamepad2.left_stick_y * 0.115;
                }
            }
            // right stick button retracts and sets timers in RETRIEVE,
            if (gamepad2.right_stick_button){
                if (currentState == states.RETRIEVE){
                    YawTimer = getRuntime();
                    PitchTimer = getRuntime();
                }
            }


            // activates RETRIEVE, if tapped again flips pitch down
            if (gamepad2.dpad_up){
                if (currentState == states.RETRIEVE && (getRuntime() - dpadUpTimer) > 0.3){
                    dpadUpTimer = getRuntime();
                    if(pitchPos != PITCH_MAX && currentState != states.ROAM) {
                        pitchPos = PITCH_MAX;
                    } else {
                        pitchPos = PITCH_MID;
                    }
                } else {
                    currentState = states.RETRIEVE;
                    extendPos = EXTEND_MAX;
                }
            }
            // activates ROAM
            if (gamepad2.dpad_down){
                currentState = states.ROAM;
                pitchPos = PITCH_MID;
                extendPos = EXTEND_MIN;
                inClawYaw = IN_YAW_MIN;
            }

            // activates SCORE & toggles outTake
            if (gamepad2.b){
                if (currentState == states.SCORE && (getRuntime() - bTimer) > 0.25) {
                    bTimer = getRuntime();
                    if (outTakePos != OUTTAKE_MIN) {
                        outTakePos = OUTTAKE_MIN;
                        outClawYaw = 1;
                        slide = 1.5;
                    } else if (outTakePos != OUTTAKE_MID) {
                        outTakePos = OUTTAKE_MID;
                        slide = 0; slideFudge = 0;
                        outClawYaw = 0;
                    }
                } else {
                    currentState = states.SCORE;
                }
            }

            if (currentState == states.RETRIEVE){
                outClawYaw = 0;
                outTakePos = OUTTAKE_MID;
            } else if (currentState == states.SCORE){
                extendPos = 0;
                pitchPos = PITCH_MIN;
            }

            // TIMERS
            if (YawTimer != 0 && (getRuntime() - YawTimer) > 0.25){
                inClawYaw = 0;
                YawTimer = 0;
            }
            if (PitchTimer != 0 && (getRuntime() - PitchTimer) > 0.1){
                pitchPos = 0.3;
                PitchTimer = 0;
            }

            robot.setExtensionPos(extendPos);
            robot.setInClawYaw(inClawYaw);
            // direct drive
            robot.setSlidePosition((slide*robot.TICKS_PER_INCH) + (slideFudge* robot.TICKS_PER_INCH));
            robot.setInClawPitchCustom(pitchPos);
            robot.setOutTakeCustom(outTakePos);
            robot.setOutClawYaw(outClawYaw);

            pinchTimer++;
            outPinchTimer++;
            sleep(25);
        }
    }
}
