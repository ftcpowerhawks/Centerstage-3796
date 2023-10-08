package org.firstinspires.ftc.teamcode.subsystem;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.configVar;

public class Elevator extends SubsystemBase {
    private final HardwareMap hardwareMap;
    private final DcMotorEx elevatorLeftMotor;
    private final DcMotorEx elevatorRightMotor;

    private final ServoEx clawRotationChanger;
    private final ServoEx clawServo;

    public BasicPID controller;

    public boolean scored = false;

    public Elevator(HardwareMap hardwareMap, double Kp, double Ki, double Kd) {
        this.hardwareMap = hardwareMap;

        elevatorLeftMotor = hardwareMap.get(DcMotorEx.class, "Left Elevator");
        elevatorRightMotor = hardwareMap.get(DcMotorEx.class, "Right Elevator");

        elevatorLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawRotationChanger = hardwareMap.get(ServoEx.class, "Claw Spin Servo");
        clawServo = hardwareMap.get(ServoEx.class, "Claw Servo");


        PIDCoefficients coefficients = new PIDCoefficients(Kp,Ki,Kd);
        BasicPID controller = new BasicPID(coefficients);


    }
    @Override
    public void register() {
        super.register();
    }
    @Override
    public void periodic() {

    }

    //Controls
    public void elevatorRunToSet3() {
        indexpixel();
        while (!scored) {
            elevatorLeftMotor.setPower(gotopos(100, elevatorLeftMotor.getCurrentPosition()));
            elevatorRightMotor.setPower(gotopos(100, elevatorRightMotor.getCurrentPosition()));
            if (clawRotationChanger.getPosition() - configVar.clawRotationScoringPos > 2) {
                clawtoscoringpos();
            }
        }
        scored = false;
    }
    public void elevatorRunToSet2() {
        indexpixel();
        while (!scored) {
            elevatorLeftMotor.setPower(gotopos(50, elevatorLeftMotor.getCurrentPosition()));
            elevatorRightMotor.setPower(gotopos(50, elevatorRightMotor.getCurrentPosition()));
            if (clawRotationChanger.getPosition() - configVar.clawRotationScoringPos > 2) {
                clawtoscoringpos();
            }
        }
        scored = false;
    }
    public void elevatorRunToSet1() {
        indexpixel();
        while (!scored) {
            elevatorLeftMotor.setPower(gotopos(30, elevatorLeftMotor.getCurrentPosition()));
            elevatorRightMotor.setPower(gotopos(30, elevatorRightMotor.getCurrentPosition()));
            if (clawRotationChanger.getPosition() - configVar.clawRotationScoringPos > 2) {
                clawtoscoringpos();
            }
        }
        scored = false;
    }
    public void elevatordown() {
        clawtoindexpos();
        while (5 < Math.abs(elevatorLeftMotor.getCurrentPosition() - 0)) {
            elevatorLeftMotor.setPower(gotopos(0, elevatorLeftMotor.getCurrentPosition()));
            elevatorRightMotor.setPower(gotopos(0, elevatorRightMotor.getCurrentPosition()));
        }
    }
    public void stop() {
        elevatorLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        elevatorRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        setPower(0.0);
    }
    public void setPower(double Power) {
        elevatorLeftMotor.setPower(Power);
        elevatorRightMotor.setPower(Power);
    }
    /** Set claw ready to score the pixel **/
    public void clawtoscoringpos() {
        clawRotationChanger.turnToAngle(configVar.clawRotationScoringPos);
    }
    /** Set claw ready to pick up a pixel from the intake **/
    public void clawtoindexpos() {
        clawRotationChanger.turnToAngle(0);
        openclaw();
    }
    /** Open the claw **/
    public void openclaw() {
        clawServo.turnToAngle(120);
    }
    /** Close the claw **/
    public void indexpixel() {
        clawServo.turnToAngle(0);
    }
    /** Recheck we are in the right position and then score on the board. **/
    public void scoreonboard() {
        clawRotationChanger.turnToAngle(configVar.clawRotationScoringPos);
        openclaw();
        scored = true;
    }
    public double gotopos(double target, double state) {
        double output = controller.calculate(target, state);
        return output;
    }
}