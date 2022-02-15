%% LIMPEZA DO AMBIENTE
%close all;
clear;
clc;

%% INTERFACE COM O USUÁRIO
promptMessage = sprintf(['\n\t########################## ROBÔ ESCRITOR ###########################', ...
    '\n\t#   Instruções:                                                    #', ...
    '\n\t#   > Escreva uma palavra de tamanho 1 a 6 utilizando              #', ...
    '\n\t#     somente letras dentro do seguinte conjunto:                  #', ...
    '\n\t#	   Letras = {A, B, C, D, E, F, G, H}                            #', ...
    '\n\t#   > Após a validação da palavra, o robô irá escrevê-la           #', ...
    '\n\t#     no quadro                                                    #', ...
    '\n\t####################################################################', ...
    '\n\n\tEntre com a palavra: ']);

palavra = input(promptMessage, 's');
%palavra = 'G'; 
if strlength(palavra) > 6
    disp('A palavra nao pode conter mais que 6 letras')
    return;
end
%%%%%%%% CRIAR tratativa da string

%% VARIÁVEIS GLOBAIS
global RoboEscritor Quadro CenarioEscrita Nuvem 
global K alpha deltaT N
global altura_letra largura_letra

%% PARAMETRIZAÇÃO DO QUADRO
largura_quadro = 1.2;
altura_quadro = 0.8;
espessura_quadro = 0.1;
dimensoes_quadro = [largura_quadro espessura_quadro altura_quadro];
% Posicionamento do quadro 
% Frame posicionado no centro geométrico do paralelepípedo
quadro_pos_x = 0;
quadro_pos_y = 0.83;
quadro_pos_z = 0.5;
loc_quadro = [quadro_pos_x; quadro_pos_y; quadro_pos_z];
% Cria quadro
Quadro = Paralelepipedo(Robo.desl(loc_quadro), ...
                   dimensoes_quadro, [0.9 0.9 0.9], 1);
% Centro da face do quadro
coords_frame_quadro = loc_quadro;
coords_frame_quadro(2) = quadro_pos_y - espessura_quadro / 2; 
FrameQuadro = Eixo(Robo.desl(coords_frame_quadro), ...
                   0.1, {'xQ', 'yQ', 'zQ'});
%% PARAMETRIZAÇÃO DAS LETRAS
qtd_letras = length(palavra);
fator_escala_letra = 1;
altura_letra = altura_quadro*0.4*(fator_escala_letra/qtd_letras);
largura_letra = largura_quadro*0.2*(fator_escala_letra/qtd_letras);

z_start_letras = loc_quadro(3) - altura_letra/2;
x_start_letras = get_Xstart_letras(qtd_letras, largura_letra, quadro_pos_x);
% Vetor de letras a serem desenhadas no quadro
letras = split(upper(palavra), '');

%% SETUP SIMULAÇÃO
% Criação do robo Kuka KR5
RoboEscritor = Robo.Cria_KukaKR5();

% Adiciona objetos ao cenário
CenarioEscrita = Cenario(RoboEscritor);
CenarioEscrita.adicionaobjetos(Quadro);
CenarioEscrita.adicionaobjetos(Nuvem);
CenarioEscrita.adicionaobjetos(FrameQuadro);
% Parâmetros gerais utilizados durante a simulação
K = diag([2 2 2 5]); % Ganho K
tau = 1 / max(max(K)); % Constante de tempo do maior K
deltaT = 0.01; % Stepsize simulação
N = 100; % Número de iterações da simulação 
max_sim_iter = round(5*(1/max(max(K))) / deltaT); % Tempo equivalente a 5*tau
q = RoboEscritor.q; % Configuração atual q
pose_inicial = RoboEscritor.cinematicadir(q, 'efetuador');
alpha = 0.005; % Coeficiente de amortecimento (pseudoinversa amortecida)
CenarioEscrita.desenha();
%% SIMULAÇÃO

for i_L=1:qtd_letras
    letra = cell2mat(letras(i_L+1));
    posicaoInicial = [x_start_letras(i_L), ...
                      coords_frame_quadro(2) - 0.01, ... 
                      z_start_letras];
    
    if letra == 'A'    
        escreveLetraA(N, posicaoInicial, [0; 1; 0]);
    elseif letra == 'B'
        escreveLetraB(N, posicaoInicial, [0; 1; 0]);
    elseif letra == 'C'
        escreveLetraC(3*N, posicaoInicial, [0; 1; 0]);
    elseif letra == 'D'    
        escreveLetraD(N, posicaoInicial, [0; 1; 0]);
    elseif letra == 'E'    
        escreveLetraE(N, posicaoInicial, [0; 1; 0]);
    elseif letra == 'F'
        escreveLetraF(N, posicaoInicial, [0; 1; 0]);
    elseif letra == 'G'
        escreveLetraG(N, posicaoInicial, [0; 1; 0]);
    elseif letra == 'H'
        escreveLetraH(N, posicaoInicial, [0; 1; 0]);
    else
        disp('A palavra não pode conter letras diferentes de A, B, C, D, E, F, G ou H.')
        return;
    end
end

% Volta para a posição inicial após escrever
simulaRobo(80, pose_inicial(1:3, 4), [0; 1; 0], false, false, true);

% Remove frame do centro do quadro para melhor visualização da escrita
CenarioEscrita.retiraobjeto(FrameQuadro);
CenarioEscrita.desenha();



%% FUNÇÕES AUXILIARES

% Retorna os coeficientes da reta com base em dois pontos
reta = @(n) pinit + (pfim - pinit)*(n/N);
%%
function escreveLetraA(N, posicaoInicial, oriz_des)
    global altura_letra largura_letra CenarioEscrita 
    
    NuvemContornoA = NuvemPontos([],[],[],[0 0 1],'-');
    NuvemMeioA = NuvemPontos([],[],[],[0 0 1],'-');
    
    CenarioEscrita.adicionaobjetos(NuvemContornoA);
    CenarioEscrita.adicionaobjetos(NuvemMeioA);
    
    % Valor sempre constante
    y_des = posicaoInicial(2);
    
    % Posiciona efetuador na posição inicial de escrita da letra A
    simulaRobo(500, posicaoInicial, oriz_des, false, false, true);

    % Reta crescente de A --> /
    pstart = posicaoInicial;
    x_end = pstart(1) + largura_letra/2;
    z_end = pstart(3) + altura_letra;
    pend = [x_end y_des z_end];
    traj_reta = criaTrajReta(pstart, pend, N);
    [r_hist, u_hist] = simulaRobo(N, traj_reta, oriz_des, ...
                                  NuvemContornoA, true, false);
                              
    % Reta decrescente de A --> \
    pstart = pend;
    x_end = posicaoInicial(1) + largura_letra;
    z_end = posicaoInicial(3);
    pend = [x_end y_des z_end];
    traj_reta = criaTrajReta(pstart, pend, N);
    simulaRobo(N, traj_reta, oriz_des, NuvemContornoA, true, false);
    % Retira pincel do quadro e o posiciona na posição de inicio
    % do traço horizontal de A
    retiraPincelQuadro(pend, 0.05);
    % Posição relativa do traço (50% da altura da letra) 
    scale_traco = 0.5;
    % Semelhança de triângulos
    deltaX_traco = scale_traco * (largura_letra/2);
    deltaZ_traco = scale_traco * altura_letra;
    x_des = posicaoInicial(1) + deltaX_traco;
    z_des = posicaoInicial(3) + deltaZ_traco;
    p_des = [x_des y_des z_des];
    simulaRobo(500, p_des, oriz_des, false, false, true);
    
    % Traço horizontal em A (meio)
    pstart = p_des;
    x_end = posicaoInicial(1) + largura_letra - deltaX_traco;
    z_end = z_des;
    pend = [x_end y_des z_end];
    traj_reta = criaTrajReta(pstart, pend, N);
    simulaRobo(N, traj_reta, oriz_des, NuvemMeioA, true, false);
    disp("Letra A desenhada.");
end

function escreveLetraB(N, posicaoInicial, oriz_des)
    global altura_letra CenarioEscrita
    
    NuvemContornoB = NuvemPontos([],[],[],[0 0 1],'-');
    CenarioEscrita.adicionaobjetos(NuvemContornoB);
   
    % Valor sempre constante
    y_des = posicaoInicial(2);
    
    % Posiciona efetuador na posição inicial de escrita da letra B
    simulaRobo(500, posicaoInicial, oriz_des, false, false, true);
    
    % Reta vertical de B
    pstart = posicaoInicial;
    pend = pstart;
    pend(3) = pend(3) + altura_letra;
    traj_reta = criaTrajReta(pstart, pend, N);
    simulaRobo(N, traj_reta, oriz_des, NuvemContornoB, true, false);
    
    % Parametrização semicircunferências
    ratio_circ_maior = 0.55;
    ratio_circ_menor = 1 - ratio_circ_maior;
    diam_circ_maior = ratio_circ_maior * altura_letra;
    diam_circ_menor = ratio_circ_menor * altura_letra;
    
    % Semicircunferência menor de B
    x_centro = posicaoInicial(1);
    y_centro = y_des;
    z_centro = posicaoInicial(3) + diam_circ_maior + (diam_circ_menor / 2);
    centro = [x_centro y_centro z_centro];
    ang_start = pi/2;
    ang_end = -pi/2;
    raio = diam_circ_menor / 2;
    traj_circular = criaTrajCircular(ang_start, ang_end, ...
                                    'h', raio, centro, N);
    simulaRobo(N, traj_circular, oriz_des, NuvemContornoB, true, false);
    
    % Semicircunferência maior de B
    x_centro = posicaoInicial(1);
    y_centro = y_des;
    z_centro = posicaoInicial(3) + (diam_circ_maior / 2);
    centro = [x_centro y_centro z_centro];
    ang_start = pi/2;
    ang_end = -pi/2;
    raio = diam_circ_maior / 2;
    traj_circular = criaTrajCircular(ang_start, ang_end, ...
                                    'h', raio, centro, N);
    simulaRobo(N, traj_circular, oriz_des, NuvemContornoB, true, false);
    
    disp("Letra B desenhada");
end

function escreveLetraC(N, posicaoInicial, oriz_des)
    global altura_letra largura_letra CenarioEscrita

    NuvemC = NuvemPontos([],[],[],[0 0 1],'-');
    CenarioEscrita.adicionaobjetos(NuvemC);
    
    % Valor sempre constante
    y_des = posicaoInicial(2);
    
    % Posiciona efetuador na posição inicial de escrita da letra C
    % Posição inicial: Centrado no centro geométrica da letra com base
    % na largura e na altura, raio r referente a metade da largura no
    % ângulo de 45º (pi/4)
    raio = largura_letra / 2;
    ang_start = pi/4;
    ang_end = 7*pi/4;
    
    pos_des_x = posicaoInicial(1) + largura_letra/2 + raio*cos(ang_start);
    pos_des_z = posicaoInicial(3) + altura_letra/2 + raio*sin(ang_start);
    pos_des = [pos_des_x y_des pos_des_z];
    
    % Posiciona efetuador na posição inicial de escrita da letra B
    simulaRobo(500, pos_des, oriz_des, false, false, true);
    
    % Curva C feita no sentido anti-horário
    x_centro = posicaoInicial(1) + largura_letra/2;
    y_centro = y_des;
    z_centro = posicaoInicial(3) + altura_letra/2;
    centro = [x_centro y_centro z_centro];

    traj_circular = criaTrajCircular(ang_start, ang_end, ...
                                    'ah', raio, centro, N);
    simulaRobo(N, traj_circular, oriz_des, NuvemC, true, false);

    disp("Letra C desenhada.");
end

function escreveLetraD(ksim, posicaoInicial, oriz_des)
    global CenarioEscrita altura_letra
    t = sym('t');

    NuvemD = NuvemPontos([],[],[],[0 0 1],'-');
    CenarioEscrita.adicionaobjetos(NuvemD);
    CenarioEscrita.desenha();
    pause(3);
    % Valor sempre constante
    y_des = posicaoInicial(2);
    % Posiciona efetuador na posição inicial de escrita da letra D
    simulaRobo(65, posicaoInicial, oriz_des, false, false)
    
    % Perna vertical de D (subindo) --> | 
    pos_horizontal_sup = [posicaoInicial(1) y_des t];
    simulaRobo(70, pos_horizontal_sup, oriz_des, NuvemD, true);
    
    % Segmento de reta direito do D
    raio = altura_letra/2;
     pos_des_x = posicaoInicial(1) + raio*cos(-t + pi/2);
    pos_des_z =  (posicaoInicial(3) + raio) + raio*sin(-t + pi/2);
    pos_horizontal_sup = [pos_des_x y_des pos_des_z];
    [r_hist, u_hist] = simulaRobo(320, pos_horizontal_sup, oriz_des, NuvemD, true);

     disp("Letra D desenhada");
end

function escreveLetraE(ksim, posicaoInicial, oriz_des)
    global altura_letra largura_letra CenarioEscrita
    t = sym('t');
    
    NuvemContornoE = NuvemPontos([],[],[],[0 0 1],'-');
    NuvemMeioE = NuvemPontos([],[],[],[0 0 1],'-');
    NuvemBaseE = NuvemPontos([],[],[],[0 0 1],'-');
    
    CenarioEscrita.adicionaobjetos(NuvemContornoE);
    CenarioEscrita.adicionaobjetos(NuvemMeioE);
    CenarioEscrita.adicionaobjetos(NuvemBaseE);
    
    % Valor sempre constante
    y_des = posicaoInicial(2);
    % Posiciona efetuador na posição inicial de escrita da letra E
    simulaRobo(ksim + 10, posicaoInicial, oriz_des, false, false)
    
    % Perna vertical de E (subindo) --> | 
    pos_horizontal_sup = [posicaoInicial(1) y_des t];
    simulaRobo(70, pos_horizontal_sup, oriz_des, NuvemContornoE, true);
    
    % Linha horizontal superior do E
    pos_des_z =  posicaoInicial(3) + altura_letra;
    pos_horizontal_sup = [t y_des pos_des_z];
    simulaRobo(20, pos_horizontal_sup, oriz_des, NuvemContornoE, true);

    % Linha horizontal superior do E (voltando) 
    pos_des_z =  posicaoInicial(3) + altura_letra;
    pos_horizontal_sup = [-t y_des pos_des_z];
    simulaRobo(20, pos_horizontal_sup, oriz_des, NuvemContornoE, true);

    % Posiciona efetuador na posição do meio do E
    pos_des_z =  posicaoInicial(3) + altura_letra/2;
    pos_des_x = posicaoInicial(1) + largura_letra;
    pos_horizontal_meio = posicaoInicial;
    pos_horizontal_meio(1) = pos_des_x;
    pos_horizontal_meio(3) = pos_des_z;
    simulaRobo(ksim + 10, pos_horizontal_meio, oriz_des, false, false);

    % Linha horizontal mediana do E
    pos_des_z =  posicaoInicial(3) + altura_letra / 2;
    pos_horizontal_sup = [-t y_des pos_des_z];
    simulaRobo(20, pos_horizontal_sup, oriz_des, NuvemMeioE, true);

    % Posiciona efetuador na posição inferior do E
    pos_des_z =  posicaoInicial(3);
    pos_des_x = posicaoInicial(1) + largura_letra;
    pos_horizontal_meio = posicaoInicial;
    pos_horizontal_meio(1) = pos_des_x;
    pos_horizontal_meio(3) = pos_des_z;
    simulaRobo(ksim + 10, pos_horizontal_meio, oriz_des, false, false);

    % Linha horizontal inferior do E
    pos_des_z =  posicaoInicial(3);
    pos_horizontal_sup = [-t y_des pos_des_z];
    simulaRobo(20, pos_horizontal_sup, oriz_des, NuvemBaseE, true);

    disp("Letra E desenhada.");  
end

function escreveLetraF(ksim, posicaoInicial, oriz_des)
    global altura_letra largura_letra CenarioEscrita
    t = sym('t');
    
    NuvemContornoF = NuvemPontos([],[],[],[0 0 1],'-');
    NuvemMeioF = NuvemPontos([],[],[],[0 0 1],'-');
    
    CenarioEscrita.adicionaobjetos(NuvemContornoF);
    CenarioEscrita.adicionaobjetos(NuvemMeioF);
    
    % Valor sempre constante
    y_des = posicaoInicial(2);
    % Posiciona efetuador na posição inicial de escrita da letra E
    simulaRobo(ksim + 10, posicaoInicial, oriz_des, false, false)
    
    % Perna vertical do F (subindo) --> | 
    pos_horizontal_sup = [posicaoInicial(1) y_des t];
    simulaRobo(70, pos_horizontal_sup, oriz_des, NuvemContornoF, true);
    
    % Linha horizontal superior do F
    pos_des_z =  posicaoInicial(3) + altura_letra;
    pos_horizontal_sup = [t y_des pos_des_z];
    simulaRobo(20, pos_horizontal_sup, oriz_des, NuvemContornoF, true);

    % Linha horizontal superior do F (voltando) 
    pos_des_z =  posicaoInicial(3) + altura_letra;
    pos_horizontal_sup = [-t y_des pos_des_z];
    simulaRobo(20, pos_horizontal_sup, oriz_des, NuvemContornoF, true);

    % Posiciona efetuador na posição do meio do F
    pos_des_z =  posicaoInicial(3) + altura_letra/2;
    pos_des_x = posicaoInicial(1) + largura_letra;
    pos_horizontal_meio = posicaoInicial;
    pos_horizontal_meio(1) = pos_des_x;
    pos_horizontal_meio(3) = pos_des_z;
    simulaRobo(ksim + 10, pos_horizontal_meio, oriz_des, false, false);

    % Linha horizontal mediana do F
    pos_des_z =  posicaoInicial(3) + altura_letra / 2;
    pos_horizontal_sup = [-t y_des pos_des_z];
    simulaRobo(20, pos_horizontal_sup, oriz_des, NuvemMeioF, true);

    disp("Letra F desenhada.");  

end

function escreveLetraG(ksim, posicaoInicial, oriz_des)
    global altura_letra largura_letra CenarioEscrita
    t = sym('t');

    NuvemContornoG = NuvemPontos([],[],[],[0 0 1],'-');
    NuvemAlturaG = NuvemPontos([],[],[],[0 0 1],'-');

    CenarioEscrita.adicionaobjetos(NuvemContornoG);
    CenarioEscrita.adicionaobjetos(NuvemAlturaG);

    % Valor sempre constante
    y_des = posicaoInicial(2);
    x_des = posicaoInicial(1) + 0.6*largura_letra;
    z_des = posicaoInicial(3) + altura_letra/2;
    pos_des_inicio = [x_des y_des z_des];

    % Posiciona efetuador na posição inicial de escrita da letra G
    simulaRobo(ksim + 10, posicaoInicial, oriz_des, false, false)

    % Perna vertical de G (subindo) --> |
    pos_horizontal_sup = [posicaoInicial(1) y_des t];
    simulaRobo(70, pos_horizontal_sup, oriz_des, NuvemAlturaG, true);

    % Linha horizontal superior do G
    pos_des_z = posicaoInicial(3) + altura_letra;
    pos_horizontal_sup = [t y_des pos_des_z];
    simulaRobo(20, pos_horizontal_sup, oriz_des, NuvemAlturaG, true);

    % Posiciona efetuador na posição inicial de escrita da letra G
    simulaRobo(ksim, pos_des_inicio, oriz_des, false, false);

    % Meio de G (esquerda para direita) --> --
    pos_des = [t y_des z_des];
    simulaRobo(15, pos_des, oriz_des, NuvemContornoG, true);

    % Lateral direita de G (meio para baixo) --> |
    x_des_lat_dir = posicaoInicial(1) + largura_letra;
    pos_des = [x_des_lat_dir y_des -t];
    simulaRobo(15, pos_des, oriz_des, NuvemContornoG, true);

    % Base do G (direita para esquerda) --> --
    pos_des = [-t y_des posicaoInicial(3)];
    simulaRobo(30, pos_des, oriz_des, NuvemContornoG, true);

    disp("Letra G desenhada.");

end

function escreveLetraH(ksim, posicaoInicial, oriz_des)
    global altura_letra largura_letra CenarioEscrita
    t = sym('t');
    
    NuvemEsquerdaH = NuvemPontos([],[],[],[0 0 1],'-');
    NuvemDireitaH = NuvemPontos([],[],[],[0 0 1],'-');
    NuvemMeioH = NuvemPontos([],[],[],[0 0 1],'-');
    
    CenarioEscrita.adicionaobjetos(NuvemEsquerdaH);
    CenarioEscrita.adicionaobjetos(NuvemDireitaH);
    CenarioEscrita.adicionaobjetos(NuvemMeioH);
    
    % Valor sempre constante
    y_des = posicaoInicial(2);
    % Posiciona efetuador na posição inicial de escrita da letra H
    simulaRobo(ksim + 10, posicaoInicial, oriz_des, false, false)
    
    % Perna vertical de H (subindo) --> | 
    pos_horizontal_sup = [posicaoInicial(1) y_des t];
    simulaRobo(80, pos_horizontal_sup, oriz_des, NuvemEsquerdaH, true);

    % Posiciona efetuador na posição do meio de H
    pos_des_z =  posicaoInicial(3) + altura_letra/2;
    pos_des_x = posicaoInicial(1);
    pos_horizontal_meio = posicaoInicial;
    pos_horizontal_meio(1) = pos_des_x;
    pos_horizontal_meio(3) = pos_des_z;
    simulaRobo(50, pos_horizontal_meio, oriz_des, false, false);
    
    % Linha horizontal do H
    pos_des_z =  posicaoInicial(3) + altura_letra/2;
    pos_horizontal_sup = [t y_des pos_des_z];
    simulaRobo(30, pos_horizontal_sup, oriz_des, NuvemMeioH, true);

    % Posiciona efetuador na posição em cima da segunda perna de H
    pos_des_z =  posicaoInicial(3) + altura_letra;
    pos_des_x = posicaoInicial(1) + largura_letra;
    pos_horizontal_meio = posicaoInicial;
    pos_horizontal_meio(1) = pos_des_x;
    pos_horizontal_meio(3) = pos_des_z;
    simulaRobo(40, pos_horizontal_meio, oriz_des, false, false);

    % Perna vertical de H (descendo) --> |
    pos_des_x = posicaoInicial(1) + largura_letra;
    pos_horizontal_inf = [pos_des_x y_des -t];
    simulaRobo(50, pos_horizontal_inf, oriz_des, NuvemDireitaH, true);

    disp("Letra H desenhada.");
end

function [r_hist, u_hist] = simulaRobo(N, p_des, oriz_des,...
                                       Nuvem, desenha, early_stop)
    global RoboEscritor CenarioEscrita K alpha deltaT Quadro
    % Função de raiz quadrada com sinal
    f = @(x) sign(x).*sqrt(abs(x));
    q = RoboEscritor.q;
    r_hist = [];
    u_hist = [];
    early_stop_tol = 1e-3;
    
    for n = 1:N
        % Calcula o tempo atual
        ti = (n-1);
        % Vetor de referência de posição
        px_des = p_des(1);
        py_des = p_des(2);
        pz_des = p_des(3);
        % Configuração atual do robô
        [J,CD] = RoboEscritor.jacobianageo(RoboEscritor.q, 'efetuador');
        px = CD(1,4);
        py = CD(2,4);
        pz = CD(3,4);
        oriz = CD(1:3,3);
        % Vamos calcular o vetor de tarefa, um vetor COLUNA
        r = [px-px_des; py-py_des; pz-pz_des; 1-(oriz_des)'*oriz];
        %Vamos calcular a Jacobiana da tarefa:
        Jr = [J(1:3,:); (oriz_des)'*Robo.matrizprodv(oriz)*J(4:6,:)];
        %Vamos calcular o termo de feedforward
        prpt = diff(sym(r));
        % Realiza substituição do instante t no vetor de posição desejada
        % caso ele possua alguma função dependente do tempo
        r = subs(sym(r), sym('t'), ti);
        prpt = subs(sym(prpt), sym('t'), ti);
        
        %Calculando a velocidade que deve ser integrada
        qdot = Robo.pinvam(Jr, alpha)*(-K*f(r)-prpt);
        %Criando o proximo k por integrac~ao
        q(:,n+1) = q(:,n)+deltaT*qdot;
        %Coloca no rob^o
        RoboEscritor.config(q(:,n+1));
        %Desenha
        %if mod(k, 3) == 0
        %Verifica se a posic~ao do efetuador esta proxima do
        %quadro para poder desenhar a figura
        CD = RoboEscritor.cinematicadir(RoboEscritor.q,'efetuador');
        xatual = CD(1,4);
%         yatual = CD(2,4);
        zatual = CD(3,4);
        if desenha
            % Projeta a nuvem de pontos diretamente no quadro
            % Coordenadas quadro 
            quadro_loc = Quadro.mth;
            % Y da face
            quadro_locy_face = quadro_loc(2,4) - (Quadro.lados(2)/2 + 0.005);
            Nuvem.px = [Nuvem.px xatual];
            Nuvem.py = [Nuvem.py quadro_locy_face];
            Nuvem.pz = [Nuvem.pz zatual]; 
        end
        CenarioEscrita.desenha();
        drawnow;
        r_hist(:,n) = r;
        u_hist(:,n) = qdot;
        if early_stop && (norm(r) < early_stop_tol)
            fprintf('\n\tEarly Stopping atingido na iteração %d', n);
            return;
        end
    end
end

function retiraPincelQuadro(patual, offset)
    pdes = patual;
    pdes(2) = pdes(2) - offset; 
    simulaRobo(200, pdes, [0; 1; 0], false, false, true);
end

function symbolic_func = criaTrajReta(pstart, pend, T)
    % T instantes de simulação entre 0 e T-1
    symbolic_func = pstart + (pend - pstart)*(sym('t')/(T-1));
end

function symbolic_func = criaTrajCircular(ang_start, ang_end, ...
                                          sentido_rot, raio, centro, T)
    % Variável simbólica temporal                     
    t = sym('t');
    % Sentido de rotação (sentido_rot)
    % h --> horario | ah --> anti-horario
    if strcmp(sentido_rot, 'h')
        sign = -1;
    elseif strcmp(sentido_rot, 'ah')
        sign = 1;
    else
        fprintf('Parametro sentido_rot não reconhecido: %s', sentido_rot);
        return;
    end
    % Passo angular de cada instante de tempo
    deltaAng = abs(ang_start - ang_end) / T;
    % Ângulo total (angulo instantaneo + fase)
    angulo = sign*(t*deltaAng) + ang_start;
    px = centro(1) + raio*cos(angulo);
    pz = centro(3) + raio*sin(angulo);
    symbolic_func = [px centro(2) pz];
end 

function startPos = get_Xstart_letras(num_letras, largura_letra, x_centro_quadro)
    % 1 letra    
    if num_letras == 1
        x_L1 = x_centro_quadro - largura_letra/2;
        startPos = [x_L1];
    % 2 letras
    elseif num_letras == 2
        x_L1 = x_centro_quadro - (1.1*largura_letra);
        x_L2 = x_centro_quadro + (0.1*largura_letra);
        startPos = [x_L1 x_L2];
    % 3 letras
    elseif num_letras == 3
        x_L1 = x_centro_quadro - (1.1*(3*largura_letra/2));
        x_L2 = x_centro_quadro - largura_letra/2;
        x_L3 = x_centro_quadro + (1.1*largura_letra/2);
        startPos = [x_L1 x_L2 x_L3];
    % 4 letras
    elseif num_letras == 4
        x_L1 = x_centro_quadro - (2.1*largura_letra);
        x_L2 = x_centro_quadro - (1.1*largura_letra);
        x_L3 = x_centro_quadro + (0.1*largura_letra);
        x_L4 = x_centro_quadro + (1.1*largura_letra);
        startPos = [x_L1 x_L2 x_L3 x_L4];
    % 5 letras    
    elseif num_letras == 5
        x_L1 = x_centro_quadro - (2.1*(3*largura_letra/2));
        x_L2 = x_centro_quadro - (1.1*(3*largura_letra/2));
        x_L3 = x_centro_quadro - largura_letra/2;
        x_L4 = x_centro_quadro + (1.1*largura_letra/2);
        x_L5 = x_centro_quadro + (2.1*largura_letra/2);
        startPos = [x_L1 x_L2 x_L3 x_L4 x_L5];
    % 6 letras
    elseif num_letras == 6    
        x_L1 = x_centro_quadro - (3.1*largura_letra);
        x_L2 = x_centro_quadro - (2.1*largura_letra);
        x_L3 = x_centro_quadro - (1.1*largura_letra);
        x_L4 = x_centro_quadro - (0.1*largura_letra);
        x_L5 = x_centro_quadro + (1.1*largura_letra);
        x_L6 = x_centro_quadro + (2.1*largura_letra);
        startPos = [x_L1 x_L2 x_L3 x_L4 x_L5 x_L6];
    else
        error('Parametrização não disponível para palavras com mais de 6 letras');
    end
end

function plotVetorTarefaHist(r_hist)
    plot(r_hist(1,:), 'LineWidth', 1);
    hold on;
    plot(r_hist(2,:), 'LineWidth', 1);
    plot(r_hist(3,:), 'LineWidth', 1);
    plot(r_hist(4,:), 'LineWidth', 1);
    legend('Posição x', 'Posição y', 'Posição z', 'Orientação y', ...,
           'Location', 'Northeast');
end
