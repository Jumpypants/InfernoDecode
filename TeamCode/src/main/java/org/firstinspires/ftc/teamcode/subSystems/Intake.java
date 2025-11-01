package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        public AutonIntakeMotor(MyRobot robotContext) {
            super(robotContext);


        }


        @Override
        protected void initialize(RobotContext robotContext) {
            IntakeMotor.set(1);

        }

        @Override
        protected boolean run(RobotContext robotContext) { //to be cont.
            return false;
        }


    }



    public class TeleOpIntake extends Task {

        public TeleOpIntake (RobotContext robotContext) {
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
