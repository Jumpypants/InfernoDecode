package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {

    private final Motor turretMotor;
    private final PIDController pid;

    private static final double P = 0.001;
    private static final double I = 0.0001;
    private static final double D = 0.004;

    private static final double MAX_POWER = 1.0;
    private static final double MIN_POWER = -1.0;

    private static final double TICKS_PER_REV = 288.0; //adjust

    private double targetPosition = 0;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = new Motor(hardwareMap, "turretMotor");
        turretMotor.setRunMode(Motor.RunMode.RawPower);
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretMotor.resetEncoder();

        pid = new PIDController(P, I, D);
    }

    public void setPosition(double newTarget) {
        double currentPosition = getCurrentPosition();
        double error = newTarget - currentPosition;

        error = (error + TICKS_PER_REV / 2) % TICKS_PER_REV - TICKS_PER_REV / 2;

        targetPosition = currentPosition + error;
        pid.setSetPoint(targetPosition);
    }

    public void update() {
        double currentPosition = getCurrentPosition();
        double power = pid.calculate(currentPosition);
        power = Math.max(Math.min(power, MAX_POWER), MIN_POWER);
        turretMotor.set(power);
    }

    public double getCurrentPosition() {
        return turretMotor.getCurrentPosition();
    }

    public void stop() {
        turretMotor.set(0);
    }
}
