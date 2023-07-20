/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleoperados.arcadedrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Arcade Drive", group = "Linear Opmode")
public class ArcadeDrive extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime(); // Objeto que conta tempo
    private static final double CPR = 1140; // Variável que guarda o Count Per Revolution do motor

    @Override
    public void runOpMode() {
        // Imprime a mensagem de que o robô iniciou
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Cria as variáveis de motores
        DcMotor motorEsquerda = hardwareMap.get(DcMotor.class, "motor_esquerda");
        DcMotor motorDireita = hardwareMap.get(DcMotor.class, "motor_direita");
        DcMotor motorEsquerdaTras = hardwareMap.get(DcMotor.class, "motor_esquerdaTras");
        DcMotor motorDireitaTras = hardwareMap.get(DcMotor.class, "motor_direitaTras");

        // Define a direção dos motores
        motorEsquerda.setDirection(DcMotor.Direction.REVERSE);
        motorDireita.setDirection(DcMotor.Direction.FORWARD);

        // Espera o start da Driver Station
        waitForStart();
        runtime.reset();

        // Loop que roda enquanto não desativarmos o código na DS
        while (opModeIsActive()) {

            // Variáveis que guardam a potência do motor
            double powerEsquerda;
            double powerDireita;

            /*
             * Aqui calculamos qual deve ser o Output dos nossos motores para que possamos digirir
             * O analógico esquerdo controla o robõ para frente e para trás
             * enquanto o direito controla o giro do robô
             */
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            powerEsquerda = drive + turn;
            powerDireita = drive - turn;

            // Atribuimos o valor máximo a variável max
            double max = Math.max(powerEsquerda, powerDireita);

            // Caso o valor seja maior que 1 fazemos uma nivelação nas velocidades, já que o motor
            // tem uma saída máxima de -1 a 1.
            if (Math.abs(max) > 1) {
                powerEsquerda /= max;
                powerDireita /= max;
            }

            // Manda as variáveis (entre [-1, 1]) para os motores individualmente
            motorEsquerda.setPower(powerEsquerda);
            motorDireita.setPower(powerDireita);
            motorEsquerdaTras.setPower(powerEsquerda);
            motorDireitaTras.setPower(powerDireita);

            // Obtemos os valores dos encoders
            double posEsquerda = motorEsquerda.getCurrentPosition();
            double posDireita = motorDireita.getCurrentPosition();

            // Telemetria dos valores que escolhemos
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", powerEsquerda, powerDireita);
            telemetry.addData("Encoders (Ticks)", "left (%.2f), right (%.2f)",
                    posEsquerda, posDireita);
            telemetry.addData("Encoders (Rotacoes)", "left (%.2f), right (%.2f)",
                    converterEncoder(posEsquerda), converterEncoder(posDireita));
            telemetry.update();
        }
    }

    // Função que tem como input a posição do motor em ticks e retorna as revoluções
    public double converterEncoder(double posicaoTicks) {
        return posicaoTicks / (CPR);
    }
}
