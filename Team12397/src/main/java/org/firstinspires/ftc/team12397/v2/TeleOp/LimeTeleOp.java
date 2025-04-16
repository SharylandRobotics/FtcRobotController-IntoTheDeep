package org.firstinspires.ftc.team12397.v2.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team12397.visionSystems.LLtdc;
import org.firstinspires.ftc.team12397.v2.RobotHardware;
import org.firstinspires.ftc.team12397.visionSystems.TdcReturnObject;
@TeleOp(name="LimeTeleOp", group="Robot")

public class LimeTeleOp extends LinearOpMode {




    TdcReturnObject tdcReturn;

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        Limelight3A limelight = this.hardwareMap.get(Limelight3A.class, "limelight-rfc");
        LLtdc tdc = new LLtdc(limelight);
        Gamepad luisL = gamepad1;

        tdc.initialize(telemetry);
        telemetry.addData(">", "Hardware Initialized");

        waitForStart();
        while (opModeIsActive()) {
            if (luisL.a){
                tdc.assessEnvironment(1);
                tdcReturn = tdc.getTdcReturn();
            } else if (luisL.y){
                tdc.initialize(telemetry);
            }

            if (tdc.getScanSuccess()){
                telemetry.addData("Scan successful? ", tdc.getScanSuccess());
                telemetry.addData("Inches right/left", tdcReturn.getRobotXCorrection(DistanceUnit.INCH));
                telemetry.addData("Inches right/left", tdcReturn.getRobotYCorrection(DistanceUnit.INCH));
                telemetry.addData("Inches right/left", tdcReturn.getArmYCorrection(DistanceUnit.INCH));
                telemetry.addData(" R rads cw/ccw", tdcReturn.getYawCorrection(AngleUnit.DEGREES));
                telemetry.addData(" C rads cw/ccw", tdcReturn.getClawYawCorrection(AngleUnit.DEGREES));
                telemetry.addData("target Points: ", tdcReturn.getCornerList());
            } else {
                telemetry.addData("Scan unsuccessful :(", "");
            }
            telemetry.addData("Time since last update ms: ", (limelight.getTimeSinceLastUpdate()));
            telemetry.update();
        }
    }


}
