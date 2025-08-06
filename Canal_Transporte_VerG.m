%--------------------------------------------------------------------------
% DISCIPLINA: THP7911 - CONTROLE ADAPTATIVO PREDITIVO
% PROGRAMA DE PÓS-GRADUAÇÃO EM ENGENHARIA ELÉTRICA.
% MATRICULA: 559990
% ALUNO: Abílio Costa da Silva Júnior
% -------------------------------------------------------------------------
% Artigo Final
%--------------------------------------------------------------------------

% =========================================================================
% PASSO 1: DEFINIR O MODELO DO CANAL DE IRRIGAÇÃO
% =========================================================================

clear; clc; close all;

% --- Definição do Modelo do Sistema (Planta) ---
% Modelo de espaço de estados linear que aproxima a dinâmica do canal.
% x(k+1) = A*x(k) + B*u(k)
% y(k)   = C*x(k) + D*u(k)

A = [0.98, 0; 
     0.01, 0.97]; % Matriz de estados: descreve a dinâmica interna dos níveis.

B = [0.2, 0;    
     0.1, 0.3];  % Matriz de entrada: descreve a influência das vazões nos níveis.

C = [1, 0;      
     0, 1];     % Matriz de saída: indica que medimos diretamente os níveis de água.

D = [0, 0;   
     0, 0];     % Matriz de alimentação direta: efeito instantâneo da entrada na saída.

Ts = 10; % Tempo de amostragem (em minutos).

% Cria o objeto de espaço de estados com atrasos (representa a planta real)
plant_with_delays = ss(A, B, C, D, Ts, 'InputDelay', [2; 1], 'OutputDelay', [3; 5]);

% Absorve os atrasos para criar o modelo que o controlador MPC usará internamente
plant_for_mpc = absorbDelay(plant_with_delays);

% =========================================================================
% PASSO 2: PROJETAR O CONTROLADOR MPC
% =========================================================================

PredictionHorizon = 15;  % Horizonte de predição (quantos passos o MPC prevê o futuro)
ControlHorizon = 5;      % Horizonte de controle (quantas ações de controle são calculadas)

% Criar o objeto MPC usando o modelo com atrasos absorvidos
mpc_controller = mpc(plant_for_mpc, Ts, PredictionHorizon, ControlHorizon);

% --- Definir Pesos e Restrições ---
% Pesos na função de custo do MPC para ajuste de desempenho.
mpc_controller.Weights.MV = [0.1 0.1];     % Penaliza o uso das entradas (vazão)
mpc_controller.Weights.MVRate = [0.5 0.5]; % Penaliza mudanças rápidas nas vazões
mpc_controller.Weights.OV = [1 1.2];       % Penaliza o desvio dos níveis do setpoint

% Restrições operacionais (limites físicos do sistema).
mpc_controller.MV(1).Min = 0; mpc_controller.MV(1).Max = 5; % Limites da vazão u1
mpc_controller.MV(2).Min = 0; mpc_controller.MV(2).Max = 4; % Limites da vazão u2
mpc_controller.OV(1).Min = 0.5; mpc_controller.OV(1).Max = 2.5; % Limites do nível y1
mpc_controller.OV(2).Min = 0.8; mpc_controller.OV(2).Max = 2.0; % Limites do nível y2

% =========================================================================
% PASSO 3: SIMULAR O SISTEMA USANDO UM LOOP MANUAL E 'mpcmove'
% =========================================================================

Tsim_steps = 60; % Duração da simulação em número de passos
r = [1.8; 1.5];  % Setpoints (níveis desejados). DEVE ser um vetor coluna.

% --- Inicialização da Simulação ---
x_plant = [1; 1]; % Estado inicial da planta real ([nivel1; nivel2])
x_mpc = mpcstate(mpc_controller); % Inicializa o estado interno do controlador MPC

% Prepara vetores para armazenar os resultados da simulação
Y = []; % Armazenará as saídas (níveis)
U = []; % Armazenará as entradas (ações de controle)

last_u = [0; 0]; % Ação de controle do passo anterior (necessário para iniciar)

% --- Loop de Simulação ---
for k = 1:Tsim_steps
    % 1. Medir a saída da planta (y[k])
    y_meas = C*x_plant + D*last_u;
    
    % 2. Calcular a ação de controle ótima com 'mpcmove'
    % Sintaxe: mpcmove(controlador, estado_controlador, saida_medida, setpoint)
    u = mpcmove(mpc_controller, x_mpc, y_meas, r);
    
    % 3. Aplicar a ação de controle na planta real para obter o próximo estado
    x_plant = A*x_plant + B*u;

    % 4. Salvar os resultados do passo atual
    U = [U, u];
    Y = [Y, y_meas];
    
    % Atualizar a última ação de controle para o próximo ciclo
    last_u = u;
end

% =========================================================================
% PASSO 4: VISUALIZAR OS RESULTADOS
% =========================================================================

t = 0:Ts:(Tsim_steps-1)*Ts; % Vetor de tempo em minutos

figure('Name', 'Simulação de Controle MPC para Canal de Irrigação');

% Plotar os níveis de água (Saídas)
subplot(2,1,1);
plot(t, Y(1,:), 'b-', 'LineWidth', 1.5);
hold on;
plot(t, Y(2,:), 'r--', 'LineWidth', 1.5);
plot(t, ones(size(t))*r(1), 'b:', 'LineWidth', 2);
plot(t, ones(size(t))*r(2), 'r:', 'LineWidth', 2);
grid on;
legend('Nível Seção 1 (y1)', 'Nível Seção 2 (y2)', 'Setpoint y1', 'Setpoint y2');
title('Resposta dos Níveis de Água');
ylabel('Nível (m)');
xlabel('Tempo (min)');
ylim([0.5 2.5]);

% Plotar as ações de controle (Entradas)
subplot(2,1,2);
stairs(t, U(1,:), 'b-', 'LineWidth', 1.5); % 'stairs' visualiza melhor sinais de controle discretos
hold on;
stairs(t, U(2,:), 'r--', 'LineWidth', 1.5);
grid on;
legend('Vazão de Entrada (u1)', 'Vazão Intermediária (u2)');
title('Ações de Controle (Vazões)');
ylabel('Vazão (m³/s)');
xlabel('Tempo (min)');
ylim([-0.1 5.5]);