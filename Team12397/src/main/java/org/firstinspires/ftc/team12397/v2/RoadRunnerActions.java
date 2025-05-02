package org.firstinspires.ftc.team12397.v2;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

public class RoadRunnerActions {
    RobotHardware robotHW;
    LinearOpMode opMode;
    public RoadRunnerActions(RobotHardware robot, LinearOpMode opMode){
        robotHW = robot;
        this.opMode = opMode;
    }

    // elbow object class
    public class VerticalSlides {
        // class vars
        private DcMotorEx verticalSlideL;
        private DcMotorEx verticalSlideR;

        // class constructor & hardware mapper
        public VerticalSlides(HardwareMap hardwareMap){
            verticalSlideL = hardwareMap.get(DcMotorEx.class, "slideMotorL");
            verticalSlideR = hardwareMap.get(DcMotorEx.class, "slideMotorR");

            verticalSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
            verticalSlideR.setDirection(DcMotorSimple.Direction.REVERSE);

            verticalSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            verticalSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // actual action class/ do-er
        public class slideToPos implements Action {
            private double Tpos; // in counts

            /**
             *
             * @param position give pure ticks/counts, use the robot. variables
             */
            public slideToPos(double position){
                Tpos = position;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setSlidePosition(Tpos);
                return false;
            }

        }

        // usable method/ action class shortcut
        public Action verticalSlidesToPos(double position){
            return new slideToPos(position);
        }
    }

    public class HorizontalExtender {
        // class vars
        private Servo extenderL;
        private Servo extenderR;

        // class constructor & hardware mapper
        public HorizontalExtender(HardwareMap hardwareMap){
            extenderL = hardwareMap.get(Servo.class, "lextend");
            extenderR = hardwareMap.get(Servo.class, "rextend");
        }

        // actual action class/ do-er
        public class extenderToPosition implements Action {
            private double Tpos; // in percentage (0 to 1)
            public extenderToPosition(double position){
                // return the smaller of the two (effectively limit to maximum count)
                Tpos = position;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setExtensionPos(Tpos);
                return false;
            }

        }

        // usable method/ action class shortcut
        public Action extenderToPos(double pos){
            return new extenderToPosition(pos);
        }
    }

    public class InClawPinch {
        // class vars
        private Servo pincher;

        // class constructor & hardware mapper
        private InClawPinch(HardwareMap hardwareMap) {
            pincher = hardwareMap.get(Servo.class, "claw_pinch");
        }

        // actual action class/ do-er(s) -->
        private class CloseInPincher implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setInClawPinch(1);
                return false;
            }
        }

        private class OpenInPincher implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setInClawPinch(0);
                return false;
            }
        }
        // <--

        // usable method/ action class shortcut
        public Action close() {
            return new InClawPinch.CloseInPincher();
        }

        public Action open() {
            return new InClawPinch.OpenInPincher();
        }
    }

    public class InClawYaw {
        // class vars
        private Servo yaw;

        // class constructor & hardware mapper
        private InClawYaw(HardwareMap hardwareMap) {
            yaw = hardwareMap.get(Servo.class, "claw_yaw");
        }

        // actual action class/ do-er(s) -->
        private class RotateInYaw implements Action {
            private double Tpos;

            public RotateInYaw(double Tpos) {
                this.Tpos = Tpos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setInClawYaw(Tpos);
                return false;
            }
        }

        // usable method/ action class shortcut
        public Action rotateYaw(double pos) {
            return new InClawYaw.RotateInYaw(pos);
        }
    }

    public class InClawPitch {
        // class vars
        private Servo pitch;

        // class constructor & hardware mapper
        private InClawPitch(HardwareMap hardwareMap) {
            pitch = hardwareMap.get(Servo.class, "claw_pitch");
        }

        // actual action class/ do-er(s) -->
        private class RotateInPitch implements Action {
            private double Tpos;

            public RotateInPitch(double Tpos) {
                this.Tpos = Tpos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setInClawPitchPos(Tpos);
                return false;
            }
        }

        /**
         * @param pos 0 is facing the floor, 1 is to transfer, -1 is tucked in
         * @return
         */
        public Action rotatePitch(double pos) {
            return new InClawPitch.RotateInPitch(pos);
        }


    }
    public class In {
        public InClawPinch clawPinch = new InClawPinch(opMode.hardwareMap);
        public InClawYaw clawYaw = new InClawYaw(opMode.hardwareMap);
        public InClawPitch clawPitch = new InClawPitch(opMode.hardwareMap);

    }

    public class OutClawPinch {
        private Servo pincher;

        // class constructor & hardware mapper
        private OutClawPinch(HardwareMap hardwareMap) {
            pincher = hardwareMap.get(Servo.class, "claw_pinch");
        }

        // actual action class/ do-er(s) -->
        private class CloseOutPincher implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setOutClawPinch(1);
                return false;
            }
        }

        private class OpenOutPincher implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setOutClawPinch(0);
                return false;
            }
        }
        // <--

        // usable method/ action class shortcut
        public Action close() {
            return new OutClawPinch.CloseOutPincher();
        }

        public Action open() {
            return new OutClawPinch.OpenOutPincher();
        }
    }

    public class OutClawYaw {
        private Servo pincher;

        // class constructor & hardware mapper
        private OutClawYaw(HardwareMap hardwareMap) {
            pincher = hardwareMap.get(Servo.class, "claw_out_yaw");
        }

        // actual action class/ do-er(s) -->
        private class deafultOutYaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setOutClawYaw(0);
                return false;
            }
        }

        private class flippedOutYaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setOutClawYaw(1);
                return false;
            }
        }
        // <--

        // usable method/ action class shortcut
        public Action defaultYaw() {
            return new OutClawYaw.deafultOutYaw();
        }

        public Action flipYaw() {
            return new OutClawYaw.flippedOutYaw();
        }
    }

    public class OutTakePosition {
        private Servo pincher;

        // class constructor & hardware mapper
        private OutTakePosition(HardwareMap hardwareMap) {
            pincher = hardwareMap.get(Servo.class, "claw_pinch");
        }

        // actual action class/ do-er(s) -->
        private class dockOutTakePos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setOutTakePos(1);
                return false;
            }
        }

        private class primeOutTakePos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setOutTakePos(0);
                return false;
            }
        }

        private class releaseOutTakePos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setOutTakePos(-1);
                return false;
            }
        }

        private class customOutTakePos implements Action {

            private double Tpos;

            public customOutTakePos(double Tpos) {
                this.Tpos = Tpos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.leftOutTake.setPosition(Tpos);
                robotHW.rightOutTake.setPosition(1.01 - Tpos);
                return false;
            }
        }
        // <--

        // usable method/ action class shortcut
        public Action dockOutTake() {
            return new OutTakePosition.dockOutTakePos();
        }

        public Action prime() {
            return new OutTakePosition.primeOutTakePos();
        }

        public Action release() {
            return new OutTakePosition.releaseOutTakePos();
        }

        public Action custom(double pos) {
            return new OutTakePosition.customOutTakePos(pos);
        }
    }

    public class Out {
        public OutClawPinch clawPinch = new OutClawPinch(opMode.hardwareMap);
        public OutClawYaw clawYaw = new OutClawYaw(opMode.hardwareMap);
        public OutTakePosition takePosition = new OutTakePosition(opMode.hardwareMap);

    }
}
