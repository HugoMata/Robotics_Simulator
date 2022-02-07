%% LIMPEZA DO AMBIENTE
close all;
clear;
clc;

%% INTERFACE COM O USUÁRIO
promptMessage = sprintf(['\n\t########################## ROBÔ ESCRITOR ###########################', ...
    '\n\t#   Instruções:                                                    #', ...
    '\n\t#   > Escreva uma palavra de tamanho 1 a 6 utilizando              #', ...
    '\n\t#     somente letras dentro do seguinte conjunto:                  #', ...
    '\n\t#	   Letras = {A, B, C, D, E, F, G, H}                       #', ...
    '\n\t#   > Após a validação da palavra, o robô irá escrevê-la           #', ...
    '\n\t#     no quadro                                                    #', ...
    '\n\t####################################################################', ...
    '\n\n\tEntre com a palavra: ']);

palavra = input(promptMessage, 's');
%palavra = 'G'; 
if strlength(palavra) > 6
    disp('A palavra nao pode conter mais que 6 letras')
    exit();
end
%%%%%%%% CRIAR tratativa da string

%% VARIÁVEIS GLOBAIS
global RoboEscritor Quadro CenarioEscrita Nuvem K alpha deltaT
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
K = diag([2 2 2 10]); % Ganho K
tau = 1 / max(max(K)); % Constante de tempo do maior K
deltaT = 0.01; % Stepsize simulação
max_sim_iter = round(5*(1/max(max(K))) / deltaT); % Tempo equivalente a 5*tau
q = RoboEscritor.q; % Configuração atual q
pose_inicial = RoboEscritor.cinematicadir(q, 'efetuador');
alpha = 0.0005; % Coeficiente de amortecimento (pseudoinversa amortecida)
CenarioEscrita.desenha();

%% SIMULAÇÃO

for i_L=1:qtd_letras
    letra = cell2mat(letras(i_L+1));
    posicaoInicial = [x_start_letras(i_L), ...
                      coords_frame_quadro(2) - 0.01, ... 
                      z_start_letras];
    
    if letra == 'A'    
        escreveLetraA(max_sim_iter, posicaoInicial, [0; 1; 0]);
    elseif letra == 'B'
        escreveLetraB(max_sim_iter, posicaoInicial, [0; 1; 0]);
    elseif letra == 'C'
        escreveLetraC(max_sim_iter, posicaoInicial, [0; 1; 0]);
    elseif letra == 'D'    
        escreveLetraD(max_sim_iter, posicaoInicial, [0; 1; 0]);
    elseif letra == 'E'    
        escreveLetraE(max_sim_iter, posicaoInicial, [0; 1; 0]);
    elseif letra == 'F'
        escreveLetraF(max_sim_iter, posicaoInicial, [0; 1; 0]);
    elseif letra == 'G'
        escreveLetraG(max_sim_iter, posicaoInicial, [0; 1; 0]);
    elseif letra == 'H'
        escreveLetraH(max_sim_iter, posicaoInicial, [0; 1; 0]);
    else
        disp('A palavra não pode conter letras diferentes de A, B, C, D, E, F, G ou H.')
        exit();
    end
end

% Volta para a posição inicial após escrever
simulaRobo(40, pose_inicial(1:3, 4), [0; 1; 0], false, false);

% Remove frame do centro do quadro para melhor visualização da escrita
CenarioEscrita.retiraobjeto(FrameQuadro);
CenarioEscrita.desenha();

%% FUNÇÕES AUXILIARES
function escreveLetraA(ksim, posicaoInicial, oriz_des)
    global altura_letra largura_letra CenarioEscrita
    t = sym('t');
    
    NuvemContornoA = NuvemPontos([],[],[],[0 0 1],'-');
    NuvemMeioA = NuvemPontos([],[],[],[0 0 1],'-');
    
    CenarioEscrita.adicionaobjetos(NuvemContornoA);
    CenarioEscrita.adicionaobjetos(NuvemMeioA);
    
    % Valor sempre constante
    y_des = posicaoInicial(2);
    % Posiciona efetuador na posição inicial de escrita da letra A
    simulaRobo(ksim + 10, posicaoInicial, oriz_des, false, false)
    
    % Perna vertical de A (subindo) --> | 
    pos_horizontal_sup = [posicaoInicial(1) y_des t];
    simulaRobo(70, pos_horizontal_sup, oriz_des, NuvemContornoA, true);
    
    % Linha horizontal superior do A
    pos_des_z =  posicaoInicial(3) + altura_letra;
    pos_horizontal_sup = [t y_des pos_des_z];
    simulaRobo(20, pos_horizontal_sup, oriz_des, NuvemContornoA, true);

    % Perna vertical de A (descendo) --> | 
    pos_des_x = posicaoInicial(1) + largura_letra;
    pos_horizontal_inf = [pos_des_x y_des -t];
    simulaRobo(20, pos_horizontal_inf, oriz_des, NuvemContornoA, true);
    
    % Posiciona efetuador na posição do meio de A
    pos_des_z =  posicaoInicial(3) + altura_letra/2;
    pos_des_x = posicaoInicial(1) + largura_letra;
    pos_horizontal_meio = posicaoInicial;
    pos_horizontal_meio(1) = pos_des_x;
    pos_horizontal_meio(3) = pos_des_z;
    simulaRobo(ksim + 10, pos_horizontal_meio, oriz_des, false, false);
   
    % Linha horizontal do meio de A
    pos_horizontal_meio = [-t y_des pos_des_z];
    simulaRobo(20, pos_horizontal_meio, oriz_des, NuvemMeioA, true);
     
    disp("Letra A desenhada.");
end

function escreveLetraB(ksim, posicaoInicial, oriz_des)
    global altura_letra largura_letra CenarioEscrita
    t = sym('t');
    NuvemContornoG = NuvemPontos([],[],[],[0 0 1],'-');
    CenarioEscrita.adicionaobjetos(NuvemContornoG);
    % Valor sempre constante
    y_des = posicaoInicial(2);
    x_des = posicaoInicial(1);
    z_des = posicaoInicial(3) + altura_letra/2;
    pos_des_inicio = [x_des y_des z_des];
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
    simulaRobo(20, pos_des, oriz_des, NuvemContornoG, true);
    
    % Lateral esquerda de G (de baixo para cima) --> |
    pos_des = [posicaoInicial(1) y_des t];
    simulaRobo(100, pos_des, oriz_des, NuvemContornoG, true);
    
     disp("Letra B desenhada.");
end

function escreveLetraC(ksim, posicaoInicial, oriz_des)
    global altura_letra largura_letra CenarioEscrita
    t = sym('t');

    NuvemC = NuvemPontos([],[],[],[0 0 1],'-');
    CenarioEscrita.adicionaobjetos(NuvemC);
    
    % Valor sempre constante
    y_des = posicaoInicial(2);
    % Posiciona efetuador na posição inicial de escrita da letra C
    pos_des_x = posicaoInicial(1) + largura_letra;
    pos_des_z = posicaoInicial(3) + altura_letra;
    pos_des_inicial = [pos_des_x y_des pos_des_z];
    simulaRobo(ksim + 10, pos_des_inicial, oriz_des, NuvemC, false)
    
    % Linha horizontal superior de C
    pos_des_z = posicaoInicial(3) + altura_letra;
    pos_horizontal_sup = [-t y_des pos_des_z];
    simulaRobo(30, pos_horizontal_sup, oriz_des, NuvemC, true);

    % Perna vertical de C (descendo) --> |
    pos_des_x = posicaoInicial(1);
    pos_horizontal_inf = [pos_des_x y_des -t];
    simulaRobo(20, pos_horizontal_inf, oriz_des, NuvemC, true);
    
    % Linha horizontal inferior de C
    pos_des_z = posicaoInicial(3);
    pos_horizontal_sup = [t y_des pos_des_z];
    simulaRobo(20, pos_horizontal_sup, oriz_des, NuvemC, true);

    disp("Letra C desenhada.");
end

function escreveLetraD(ksim, posicaoInicial, oriz_des)
    global CenarioEscrita largura_letra
    t = sym('t');

    NuvemD = NuvemPontos([],[],[],[0 0 1],'-');
    CenarioEscrita.adicionaobjetos(NuvemD);
    
    % Valor sempre constante
    y_des = posicaoInicial(2);
    % Posiciona efetuador na posição inicial de escrita da letra D
    simulaRobo(ksim + 10, posicaoInicial, oriz_des, false, false)
    
    % Perna vertical de D (subindo) --> | 
    pos_horizontal_sup = [posicaoInicial(1) y_des t];
    simulaRobo(70, pos_horizontal_sup, oriz_des, NuvemD, true);
    
    % Segmento de reta direito do D
    pos_des_z =  posicaoInicial(3)*2/3;
    pos_horizontal_sup = [t y_des pos_des_z];
    simulaRobo(70, pos_horizontal_sup, oriz_des, NuvemD, true);

    % Perna horizontal do D (direita para esquerda) --> ____ 
    pos_horizontal_inf = [-t y_des posicaoInicial(3)];
    simulaRobo(60, pos_horizontal_inf, oriz_des, NuvemD, true);
    
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


% Testa se o vetor de posição desejado é uma função temporal
% function bool_return = isDynamicReference(var_ref)
%     bool_return = ~isempty(symvar(var_ref));
% end

function simulaRobo(ksim, p_des, oriz_des, Nuvem, desenha)
    global RoboEscritor Quadro CenarioEscrita deltaT K alpha
    global altura_letra largura_letra
    % Função de raiz quadrada com sinal
    f = @(x) sign(x).*sqrt(abs(x));
    q = RoboEscritor.q;
    satura_x = true;
    satura_z = true;
    for k = 1:ksim
        % Calcula o tempo atual
        ti = (k-1)*deltaT;
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
        
        %Calculando a velocidade que deve ser integrada
        qdot = Robo.pinvam(Jr, alpha)*(-K*f(r)-prpt);
        %Criando o proximo k por integrac~ao
        q(:,k+1) = q(:,k)+deltaT*qdot;
        %Coloca no rob^o
        RoboEscritor.config(q(:,k+1));
        %Desenha
        %if mod(k, 3) == 0
        %Verifica se a posic~ao do efetuador esta proxima do
        %quadro para poder desenhar a figura
        CD = RoboEscritor.cinematicadir(RoboEscritor.q,'efetuador');
        xatual = CD(1,4);
        zatual = CD(3,4);
        if desenha
            % Coordenadas quadro 
            quadro_loc = Quadro.mth;
            quadro_locx = quadro_loc(1,4);
            quadro_locz = quadro_loc(3,4);
            % Y da face
            quadro_locy_face = quadro_loc(2,4) - (Quadro.lados(2)/2 + 0.01);
            
            % Distancia do efetuador ao centro do quadro
            dist_cquadro_x =  abs(quadro_locx - xatual);
            dist_cquadro_z = abs(quadro_locz - zatual);
            tol = 1.05;
            % Satura posição em X
            if (dist_cquadro_x > tol*largura_letra/2) && satura_x
                p_des(1) = xatual;
                satura_x = false;
            end
            % Satura posição em Z
            if (dist_cquadro_z > tol*altura_letra/2) && satura_z
                p_des(3) = zatual;
                satura_z = false;
            end
            
            Nuvem.px = [Nuvem.px xatual];
            Nuvem.py = [Nuvem.py quadro_locy_face];
            Nuvem.pz = [Nuvem.pz zatual]; 
        end
        CenarioEscrita.desenha();
        drawnow;
        
    end
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