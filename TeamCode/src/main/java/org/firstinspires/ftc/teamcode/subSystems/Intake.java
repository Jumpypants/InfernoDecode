package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MyRobot;
public class Intake {
    private final Motor IntakeMotor;
    private final Gamepad gamepad1;

    private static int buttonClick;

    public Intake(HardwareMap hardwareMap, Gamepad gamepad1) {
        IntakeMotor = new Motor(hardwareMap, "IntakeMotor");
        IntakeMotor.setRunMode(Motor.RunMode.RawPower);
        IntakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        buttonClick = 0;

        this.gamepad1 = gamepad1;
    }

    public class runIntakeMotor extends Task {

        public runIntakeMotor(RobotContext robotContext, double targetPosition) {
            super(robotContext);
        }

        @Override
        protected void initialize(RobotContext robotContext) {

        }

        @Override
        protected boolean run(RobotContext robotContext) {
            buttonClick++;
            if (buttonClick == 1) {
                IntakeMotor.set(1);
            } else if (buttonClick == 2) {
                IntakeMotor.set(0);
                buttonClick = 0;
            }

            return true;
        }
    }


}
