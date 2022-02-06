%% LIMPEZA DO AMBIENTE
%close all;
clear;
clc;

%% INTERFACE COM O USUÁRIO
promptMessage = sprintf(['\n\t########################## ROBÔ ESCRITOR ###########################', ...
    '\n\t#   Instruções:                                                    #', ...
    '\n\t#   > Escreva uma palavra de tamanho 1 a 6 utilizando              #', ...
    '\n\t#     somente letras dentro do seguinte conjunto:                  #', ...
    '\n\t#	   Letras = {A, B, C, D, E, F, G, H}                           #', ...
    '\n\t#   > Após a validação da palavra, o robô irá escrevê-la           #', ...
    '\n\t#     no quadro                                                    #', ...
    '\n\t####################################################################', ...
    '\n\n\tEntre com a palavra: ']);
  
% palavra = input(promptMessage, 's');
palavra = 'A';
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
quadro_pos_y = 0.85;
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
largura_letra = largura_quadro*0.1*(fator_escala_letra/qtd_letras);

z_start_letras = loc_quadro(3) - altura_letra/2;
x_start_letras = get_Xstart_letras(qtd_letras, largura_letra, quadro_pos_x);
% Vetor de letras a serem desenhadas no quadro
letras = split(upper(palavra), '');

%% SETUP SIMULAÇÃO
% Criação do robo Kuka KR5
RoboEscritor = Robo.Cria_KukaKR5();

% Mapeia nuvem de pontos
Nuvem = NuvemPontos([],[],[],[0 0 1],'-');
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
alpha = 0.0005; % Coeficiente de amortecimento (pseudoinversa amortecida)
CenarioEscrita.desenha();

%% SIMULAÇÃO

for i_L=1:qtd_letras
    letra = cell2mat(letras(i_L+1));
    posicaoInicial = [x_start_letras(i_L), ...
                      coords_frame_quadro(2) - 0.01, ... 
                      z_start_letras];
    
    if letra == 'A'    
        escreveLetraA(max_sim_iter, posicaoInicial, [0; 1; 0])
    end
end


function escreveLetraA(ksim, posicaoInicial, oriz_des)
    global altura_letra largura_letra deltaT
    t = sym('t');
    b = sym('b');
    
    % Valor sempre constante
    y_des = posicaoInicial(2);
    % Posiciona efetuador na posição inicial de escrita da letra A
    simulaRobo(ksim + 50, posicaoInicial, oriz_des, false)
    
   
%     a = altura_letra / (largura_letra / 2);
%     
%     lin_equation = altura_letra/2 == a*t+b;
%     subs(lin_equation, t, largura_letra/2, b);
%     simul_time = solve(lin_equation);
    % Perna vertical de A --> |
    ksim_calc = altura_letra / deltaT; 
    pos_des_subida = [posicaoInicial(1) y_des t];
    simulaRobo(100, pos_des_subida, oriz_des, true);

end

% Testa se o vetor de posição desejado é uma função temporal
% function bool_return = isDynamicReference(var_ref)
%     bool_return = ~isempty(symvar(var_ref));
% end

function simulaRobo(ksim, p_des, oriz_des, desenha)
    global RoboEscritor Quadro CenarioEscrita Nuvem deltaT K alpha
    global altura_letra largura_letra
    % Função de raiz quadrada com sinal
    f = @(x) sign(x).*sqrt(abs(x));
    q = RoboEscritor.q;
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
        [~,Dist] = Quadro.calcdistponto(CD(1:3,4));
        xatual = CD(1,4);
        yatual = CD(2,4);
        zatual = CD(3,4);
        if desenha
            Nuvem.px = [Nuvem.px xatual];
            Nuvem.py = [Nuvem.py yatual];
            Nuvem.pz = [Nuvem.pz zatual];
        end
        CenarioEscrita.desenha();
        drawnow;
        if ((abs(zatual) > altura_letra/2) || (abs(xatual) > largura_letra/2)) && desenha
            break;
        end
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
 
    

    


   

  