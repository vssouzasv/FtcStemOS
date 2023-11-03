
package org.firstinspires.ftc.teamcode.comandos.motor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="Motor to sP", group="Robot")
public class ControleMotor extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor motor   = null;

    // Constantes para conversão
    static final double     COUNTS_PER_MOTOR_REV    = 560; // CPR do motor, entre no site da fabricante
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;  // Redução entre motor e roda
    
    static final double     WHEEL_DIAMETER_INCHES   = 1.96;
    double fatorDeConversao = (COUNTS_PER_MOTOR_REV  * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Iniciando");
        telemetry.update();
        // Inicia o mtoor
        motor  = hardwareMap.get(DcMotor.class, "motor");
        // Seta o motor como FORWARD
        motor.setDirection(DcMotor.Direction.FORWARD);
        // Para e reseta os encoder
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Espera o start
        waitForStart();

        // Para adicionar objetivos apenas chame a função addSetpoint(valor em polegadas)
        // Como não resetamos o encoder a cada movimento, a rotação será relativa ao motor
        // Portanto se eu avançar para 10, e quiser voltar a 0, é necessário colocar -10
        addSetpoint(10);
        sleep(1000);
        addSetpoint(-10);
    }

    public void addSetpoint(int setPoint) {
        // Define um objetivo
        int newSetPoint = motor.getCurrentPosition() + (int)(setPoint * fatorDeConversao);
        // Define uma posição alvo
        motor.setTargetPosition(newSetPoint);
        // Seta o motor para ir até a posição
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Coloca o motor para se mover
        motor.setPower(0.5);

        // Distância percorrida pelo motor
        while(motor.isBusy()) {
            telemetry.addData("Distância percorrida: ", (motor.getCurrentPosition() / fatorDeConversao));
            telemetry.update();
        }
        // Para os motores
        motor.setPower(0);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);
    }
}
