package org.firstinspires.ftc.teamcode.teleoperados.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "mecanum fieldOriented", group = "Linear TesteOp")
public class Mecanum extends LinearOpMode {
    // Objeto que conta tempo
    ElapsedTime runtime = new ElapsedTime();

    // Objeto que guarda os nossos acionadores e sensores
    HardwareClassTracao hard = new HardwareClassTracao(this);

    // Respectivamente eixos do gamepad y, x, x2 (outro analógico)
    double drive, turn, giro;

    // Vetor para acionamentos dos motores;
    private final double[] poder = new double[4];

    // Variável que guarda o ângulo do robô (radianos)
    double angle;

    @Override
    public void runOpMode() {
        // Telemetria de inicio
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Inicia o hardware do robô
        hard.hardwareGeral();

        runtime.reset();

        // Espera o botão start na Ds
        waitForStart();

        while (opModeIsActive()) {
            /*
             * drive controla a translação do robô (frente e trás)
             * turn controla a translação do robô (esquerda e direita)
             * giro controla a rotação do robô
             */
            drive = -gamepad1.left_stick_y; // Negativo por causa do gamepad
            turn = gamepad1.left_stick_x * 1.5; // Multiplicamos por um valor para melhorar o strafe
            giro = gamepad1.right_stick_x;

            /*
             * Calculo para orientação no campo
             * caso queira desativar, apenas apague ou comente a linha abaixo
             */
            fieldOriented(drive, turn);

            // Valores para movimentação com mecanum (lados espelhados)
            //Motor Esquerda Frente;
            poder[0] = drive + turn + giro;
            // Motor Esquerda trás;
            poder[1] = drive - turn + giro;
            // Motor Direita Frente;
            poder[2] = drive - turn - giro;
            // Motor Direita trás;
            poder[3] = drive + turn - giro;

            // Verificar se algum valor é maior que 1
            if (Math.abs(poder[0]) > 1 || Math.abs(poder[1]) > 1
                    || Math.abs(poder[2]) > 1 || Math.abs(poder[3]) > 1) {

                //Achar o maior valor
                double max;
                max = Math.max(Math.abs(poder[0]), Math.abs(poder[1]));
                max = Math.max(Math.abs(poder[2]), max);
                max = Math.max(Math.abs(poder[3]), max);

                //Não ultrapassar +/-1 (proporção);
                poder[0] /= max;
                poder[1] /= max;
                poder[2] /= max;
                poder[3] /= max;
            }

            // Metodo setPower que manda a potência para os motores.
            hard.motorEsquerda.setPower(poder[0]);
            hard.motorEsquerdaTras.setPower(poder[1]);
            hard.motorDireita.setPower(poder[2]);
            hard.motorDireitaTras.setPower(poder[3]);

            // Telemetria com os valores de cada roda
            telemetry.addData("Motor Esquerdo %.2f ", poder[0]);
            telemetry.addData("Motor EsquerdoTras %.2f ", poder[1]);
            telemetry.addData("Motor Direita %.2f ", poder[2]);
            telemetry.addData("Motor DireitaTras %.2f ", poder[3]);
            telemetry.addData("Orientação %.2f ", Math.toDegrees(angle));
            telemetry.addData("Tempo passado ", runtime.toString());
            telemetry.update();
        }
    }

    // Função que calcula a orientação no campo
    private void fieldOriented(double driveP, double turnP) {
        angle = Math.toRadians(gyroCalculate());
        drive = driveP * Math.cos(angle) - turnP * Math.sin(angle);
        turn = driveP * Math.sin(angle) + turnP * Math.cos(angle);
    }

    // Função que retorna a orientação do robô em graus
    private double gyroCalculate() {
        YawPitchRollAngles orientation = hard.imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
