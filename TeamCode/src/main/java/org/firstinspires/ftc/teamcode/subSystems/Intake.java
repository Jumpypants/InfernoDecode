package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MyRobot;
public class Intake {
    private final Motor IntakeMotor;

    private static boolean buttonClick;

    public Intake(HardwareMap hardwareMap) {
        IntakeMotor = new Motor(hardwareMap, "IntakeMotor");
        IntakeMotor.setRunMode(Motor.RunMode.RawPower);
        IntakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        buttonClick = false;

    }

    public class AutonIntakeMotor extends Task {
        private final double TARGET_POSITION; 
        public AutonIntakeMotor(RobotContext robotContext, double targetPosition) {
            super(robotContext);
            TARGET_POSITION = targetPosition;

        }

        @Override
        protected void initialize(RobotContext robotContext) {
            IntakeMotor.set(0);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            double currentPos = IntakeMotor.getCurrentPosition();
            if (Math.abs(TARGET_POSITION-currentPos)<10) {
                IntakeMotor.set(0);
                return true;
            }
            IntakeMotor.set(1);
            return false;
        }
    }

    public class TeleOpIntake extends Task {

        public TeleOpIntake (RobotContext robotContext, double targetPosition) {
            super(robotContext);
        }

        @Override
        protected void initialize(RobotContext robotContext) {

        }

        @Override
        protected boolean run(RobotContext robotContext) {
            if (robotContext.gamepad1.x){
                if (buttonClick == false) {
                    buttonClick = true;
                    IntakeMotor.set(1);
                } else if (buttonClick == true) {
                    IntakeMotor.set(0);
                    buttonClick = false;
                }
            }

            return true;
        }
    }


}
