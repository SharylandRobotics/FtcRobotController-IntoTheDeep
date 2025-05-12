package org.firstinspires.ftc.team12397.v2.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.team12397.v2.RoadRunnerActions;
import org.firstinspires.ftc.team12397.v2.RobotHardware;


@TeleOp(name="Field Centric (2)", group="Robot")
public class FieldCentric2 extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    private final double OUTTAKE_MAX = robot.OUTTAKE_MAX;
    private final double OUTTAKE_MIN = robot.OUTTAKE_MIN;
    private final double OUTTAKE_MID = robot.OUTTAKE_MID;

    private final double EXTEND_MAX = robot.EXTEND_MAX;
    private final double EXTEND_MID = robot.EXTEND_MID;
    private final double EXTEND_MIN = robot.EXTEND_MIN;

    private final double PITCH_MAX = robot.PITCH_MAX-0.025;
    private final double PITCH_MID = robot.PITCH_MID;
    private final double PITCH_MIN = robot.PITCH_MIN;

    private final double IN_YAW_MAX = robot.IN_YAW_MAX;
    private final double IN_YAW_MIN = robot.IN_YAW_MIN;

    private final double OUT_YAW_MAX = robot.OUT_YAW_MAX;
    private final double OUT_YAW_MIN = robot.OUT_YAW_MIN;

    private final double SLIDE_RUNG = robot.SLIDE_RUNG+0.5;
    private final double SLIDE_ALT = robot.SLIDE_ALT;

    enum states{
        RETRIEVE,
        ROAM,
        SCORE,
        SCOREB
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
        double outTakePos = OUTTAKE_MID-0.1;
        double extendPos = 0;
        double pitchPos = 0.5;
        double outClawYaw = 0;

        int pinchTimer = 0;
        boolean pinchToggle = false;

        int outPinchTimer = 0;
        boolean outPinchToggle = false;

        double OutTimer = 0;
        double OutTimer2 = 0;
        double PinchTimer = 0;
        double dpadUpTimer = 0;
        double slideUpTimer = 0;
        double bTimer = 0;

        double slide = 0;
        double slideFudge = 0;

        handle inClawPinch = handle.OPEN;
        handle outClawPinch = handle.OPEN;

        states currentState = states.ROAM;

        robot.init(true);


        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.left_stick_y == 1){
                drive = 1;
            } else {
                drive = gamepad1.left_stick_y * 0.7;
            }
            strafe = -gamepad1.left_stick_x*1.1;
            turn  =   gamepad1.right_stick_x;
            robot.driveFieldCentric(drive, strafe, turn);
            if (gamepad1.right_bumper && outTakePos == robot.OUTTAKE_ALT && slide == SLIDE_RUNG){
                slide = SLIDE_ALT;
            } else if (gamepad1.left_bumper && outTakePos == robot.OUTTAKE_ALT && slide == SLIDE_RUNG) {
                slide = SLIDE_RUNG;
                OutTimer = 0;
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
            // right stick button retracts and sets timers in SCOREB,
            if (gamepad2.right_stick_button){
                if (currentState == states.SCOREB){
                    outTakePos = OUTTAKE_MID+0.04;
                    pitchPos = robot.PITCH_TRANS;
                    extendPos = robot.EXTEND_TRANS+0.16;
                    inClawYaw = IN_YAW_MIN;
                    outClawYaw = OUT_YAW_MIN;
                    OutTimer2 = getRuntime();
                    PinchTimer = getRuntime();
                    slideUpTimer = getRuntime();
                } else {
                    currentState = states.SCOREB;
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
                    dpadUpTimer = getRuntime();
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

            // activates SCORE & toggles outTake activities
            if (gamepad2.b){
                if (currentState == states.SCORE && (getRuntime() - bTimer) > 0.25) {
                    bTimer = getRuntime();
                    if (outTakePos != robot.OUTTAKE_ALT) {
                        outTakePos = robot.OUTTAKE_ALT;
                        outClawYaw = 1;
                        slide = SLIDE_RUNG;
                    } else if (outTakePos != OUTTAKE_MID) {
                        outTakePos = OUTTAKE_MID;
                        slide = 0;
                        outClawYaw = 0;
                    }
                } else if (currentState == states.SCOREB && (getRuntime() - bTimer) > 0.25){
                    bTimer = getRuntime();
                    if (!outPinchToggle){
                        slide = 0;
                        outTakePos = robot.OUTTAKE_PARALLEL;
                        currentState = states.ROAM;
                    } else if (outTakePos != robot.OUTTAKE_BALT) {
                        outTakePos = robot.OUTTAKE_BALT;
                        outClawYaw = 0;
                    } else if (outTakePos != robot.OUTTAKE_PARALLEL) {
                        outTakePos = robot.OUTTAKE_PARALLEL;
                        outClawYaw = 0;
                    }
                } else {
                    if(currentState!= states.SCOREB){
                        currentState = states.SCORE;
                    }
                }
            }

            if (currentState == states.RETRIEVE){
                outClawYaw = 0;
            } else if (currentState == states.SCORE){
                extendPos = 0;
                pitchPos = PITCH_MIN;
            }

            // TIMERS
            if (OutTimer != 0 && (getRuntime() - OutTimer) > 0.5){
                outPinchToggle = false;
                OutTimer = 0;
            }
            if (OutTimer2 != 0 && (getRuntime() - OutTimer2) > 0.5){
                outPinchToggle = true;
                OutTimer2 = 0;
            }
            if (PinchTimer != 0 && (getRuntime() - PinchTimer) > 1){
                pinchToggle = false;
                PinchTimer = 0;
            }
            if (slideUpTimer != 0 && (getRuntime() - slideUpTimer) > 1.75){
                slide = 42.5 - 18.5;
                slideUpTimer = 0;
                outTakePos = robot.OUTTAKE_PARALLEL;
            }

            robot.setExtensionPos(extendPos);
            robot.setInClawYaw(inClawYaw);
            // direct drive
            robot.setSlidePosition((slide*robot.TICKS_PER_INCH));
            robot.setInClawPitchCustom(pitchPos);
            robot.setOutTakeCustom(outTakePos);
            robot.setOutClawYaw(outClawYaw);

            pinchTimer++;
            outPinchTimer++;
            sleep(25);
        }
    }
}
