ESPECIFICAÇÃO ARQUITETÔNICA E ESTRUTURAL COMPLETA ("AS-BUILT")

Projeto: VANT Tractor Twin-Boom (Híbrido) com Cauda Central
Plataforma de Controle: ESP32 (Arquitetura Dual-Core)
Revisão: Layout Eletrônico e Integração de Performance de Missão

1. EVOLUÇÃO ARQUITETÔNICA (O ANTIGO VS. O NOVO)

Para garantir precisão milimétrica, precisamos ter clareza total do que está mudando fisicamente e aerodinamicamente.

1.1. O Projeto Antigo (Asa Voadora Plank Pusher)

Geometria: Fuselagem central (Pod) de 22cm alinhada com o bordo de ataque. Asas com perfil Armin (corda 22cm), enflexamento de 12º e sem diedro.

Estabilização Passiva: Dependia de grandes winglets (VNT Beta-1) nas pontas das asas para evitar guinada (Yaw).

Controle: "Reflex" obrigatório (Trim UP constante) nos elevons para manter o nariz nivelado. O controle de Pitch (arfagem) e Roll (rolagem) era mixado e processado pelo Arduino no mesmo conjunto de atuadores (MG90s).

Propulsão e CG: Motor pusher (traseiro) com Up-Thrust compensatório. Bateria deslocada como lastro para manter o CG crítico a 70-72mm do bico do Pod.

1.2. O Novo Projeto (Tractor Convencional com Cauda Central)

Geometria: Asas mantêm o perfil e enflexamento. O Pod central ganha um "nariz" (extensão frontal), e a traseira recebe duas lanças (Twin-Booms com 4cm de distância) segurando uma nova empenagem tradicional.

Estabilização Passiva: Winglets removidas. Uma Deriva Vertical Central Única assume o controle passivo de Yaw com eficiência máxima, eliminando o arrasto de interferência. Sem necessidade de "Reflex" na asa principal (ganho absurdo de sustentação e perda de arrasto).

Controle: Atuadores separados. Asas apenas fazem Roll (Ailerons puros). Nova cauda faz Pitch (Profundor puro) operado por um 3º servo via linkagem rígida guiada. Processamento dividido nos 2 núcleos do ESP32.

Propulsão e CG: Motor movido para a frente (Tractor). CG recalculado para adaptação às hastes reais.

2. DECISÃO DE MATERIAIS PARA AS LANÇAS (BOOMS)

Material Escolhido: Fibra de Vidro 4mm Maciça.

Características Físicas e Estruturais:
- Rigidez à flexão ($EI_{\text{fg}}$): $1.005\text{ N}\cdot\text{m}^2$ (com $E = 40\text{ GPa}$).
- Deflexão vertical na cauda sob carga de $1.5\text{ N}$ (força típica de profundor): $9.8\text{ mm}$ de flexão vertical.
- Peso total das hastes: $27.0\text{ g}$.
- Nota de Segurança Estrutural: Devido à alta flexibilidade do material, recomenda-se limitar a velocidade de mergulho e velocidade máxima de voo da aeronave abaixo de 42 km/h para evitar flutter aerodinâmico. O material possui a vantagem de não interferir em sinais eletromagnéticos (antenas LoRa e GPS).

3. A NECESSIDADE DA EXTENSÃO FRONTAL (O NOVO "NARIZ")

O Motivo Físico (Efeito Gangorra):
A fibra de vidro maciça de 4mm e o conjunto de cauda de XPS recuado a 27cm para trás da asa tornam o avião pesado de cauda (Tail Heavy). É obrigatório manter o nariz frontal alongado em 8 a 10 cm para empurrar a bateria o máximo possível para frente, cravando o CG sem necessidade de lastro morto.

A Solução (O Nariz de XPS):

Construa um "nariz" em formato de cone truncado ou bloco em XPS com cerca de 8 a 10 cm de comprimento para frente do Bordo de Ataque.

Cole isso na frente da sua fuselagem atual.

Esculpa um compartimento interno nesse nariz para empurrar a bateria LG HG2 o máximo possível para frente.

Fixe a parede de fogo (MDF/Plástico 4x4cm) e o motor na ponta desse novo nariz.

4. REPRESENTAÇÃO VISUAL ESQUEMÁTICA (DIAGRAMAS)

4.1. VISTA SUPERIOR (Top-Down View)

                           [ HÉLICE ]
                              |  |    <-- Motor inclinado 1.5º à direita, 2 a 3º p/ baixo
                          [Motor A2212]
        NOVO NARIZ ----->  /=========\   <-- Parede de fogo 4x4cm
        (Extensão de      /  [Bat.]   \  <-- Bateria LG empurrada pro nariz
         8 a 10 cm)      /   [ESP32]   \     para equilibrar CG
                        /    [GPS/RF]   \
                       /                 \
   [LE: Sweep 12º]    |===================|    [LE: Sweep 12º]
   ===================|      [ESC]        |===================
   |      Asa         |  < NOVO CG 5.8cm  |      Asa         |
   |   Esquerda       |   [Servo Cauda]   |   Direita        |
   |                  |----(Fim do Pod)---|                  |
   +---[AILERON]------+   ||           || +---[AILERON]------+
         (Só Roll)        ||  (4 cm)   ||       (Só Roll)
                          ||           ||
Vão livre de 27 cm -----> ||           || <-- Hastes de Fibra de Vidro (4mm)
       Guias de Arame --> oo           ||
                          oo           ||
                          ||           ||
                          ||           ||
                 +--------||-----------||--------+  <-- Estabilizador Horizontal XPS (38x11cm)
   Hastes apoiam |        || [Deriva]  ||        |  <-- Deriva Vertical Central Única (Base 10cm)
  apenas 6cm aqui|        || [Central] ||        |
                              [       PROFUNDOR (36cm)      ]   <-- Só Pitch
                                ^ Arame (Aço) conecta aqui


5. RESULTADOS DOS CÁLCULOS DE AERODINÂMICA E PERFORMANCE DE MISSÃO

Com as medições exatas da bancada, extraímos os coeficientes de estabilidade e o envelope de desempenho da aeronave com a configuração de fibra de vidro.

5.1. Estabilidade Longitudinal e Geometria

Braço de Alavanca ($L_h$): 43,3 cm (A cauda distante gera enorme torque estabilizador).

Novo Ponto Neutro (NP): 12,5 cm a partir do Bordo de Ataque principal.

Volume de Cauda Horizontal ($V_h$): 0.62 (Estabilidade de arfagem inabalável).

Volume de Cauda Vertical ($V_v$): 0.053 (Autoridade direcional impecável com a nova deriva).

5.2. Dinâmica de Voo e Envelope Físico (Flight Envelope)

Massa Total Real (AUW): 600 gramas.

Carga Alar (Wing Loading): 45,3 g/dm². (Cortará o vento muito melhor e sofrerá menos com rajadas laterais).

Velocidade de Estol ($V_{stall}$): ~28.0 km/h (7.7 m/s). Apesar de mais pesado, o abandono do "Reflex" permitiu que a asa gerasse mais sustentação, reduzindo a velocidade de estol em relação ao projeto original.

Velocidade de Cruzeiro ($V_{cruise}$): Otimizada entre 38 km/h e 42 km/h.

Razão Empuxo-Peso (TWR): Utilizando motor A2212 (hélice de 10x4.5" - 1045), o empuxo será de ~650g a 700g. TWR de 1.08:1, garantindo subida em fortes ângulos de ataque.

5.3. Performance Energética e Telemetria (Missão)

Eficiência Aerodinâmica (L/D Ratio): 5,67:1. O arrasto total no cruzeiro exige apenas ~10.9 Watts de potência mecânica, traduzindo-se em ~31.2 Watts de consumo elétrico total (Motor + Aviônica).

Tempo de Voo (Endurance): Operando a Bateria LG HG2 2S (3000mAh) até uma margem segura de 80% de descarga (17,76 Wh úteis), o avião terá ~34 minutos e 15 segundos de voo contínuo em cruzeiro.

Alcance de Combate (Combat Radius): Possibilidade de afastar-se até 8 a 10 Quilômetros da base e retornar com segurança.

Orçamento de Link de RF (LoRa 433MHz): Com +20dBm de transmissão, o Link Budget total chega a ~149 dB. A atenuação a 8 km de distância é de ~103 dB, o que deixa uma sobra avassaladora (+45 dB de Fade Margin). O rádio será 100% blindado contra quedas de sinal em linha de visada.

6. PLANTA DE CORTE E GEOMETRIA

6.1. Estabilizador Horizontal + Profundor (Vista Superior)

    <----------------------- 38.0 cm (Envergadura Total) ----------------------->
    
 ^  +---------------------------------------------------------------------------+
 |  |                                                                           |
 |  |                       ESTABILIZADOR HORIZONTAL FIXO                       |
7.7 |                 (As hastes apoiam APENAS os 6 cm frontais)                |
cm  |                                                                           |
 |  |                                                                           |
 v  +===========================================================================+  <-- Linha de Hinge
 ^  | |                                                                       | |      
3.3 | |                       PROFUNDOR MÓVEL (36 cm)                         | |   
cm  | +_______________________________________________________________________+ |     
 v                                                                              
      <----------------------------- 36.0 cm --------------------------------->


6.2. Deriva Vertical Central Única (Vista Lateral)

Cortar 1 peça e colar perfeitamente centralizada (marcador dos 19 cm) no estabilizador horizontal.

               <------ 6.5 cm ------>
             +------------------------+    ^
            /                         |    |
           /                          |    |
          /                           |    |
         /                            |    |  12.0 cm
        /                             |    | (Altura Vertical)
       /                              |    |
      +-------------------------------+    v  <-- Ângulo exato de 90º
      <------------- 10.0 cm ------------->
         (Base colada no Horizontal)


6.3. Micro-Winglets Opcionais (Placas de Ponta de Asa)

Corte 2 peças em XPS e cole niveladas nas laterais das asas apenas para bloquear vórtices de ponta e proteger as asas no pouso.

               <------ 12.0 cm ------> 
             +------------------------+      ^
            /                         |      |  3.0 cm (Para cima da asa)
           /                          |      v
          +---------------------------|  <-- LINHA DA ASA
         /                            |      ^
        /                             |      |  1.5 cm (Para baixo da asa / Skid)
       +------------------------------+      v
      <----------- 22.0 cm ----------->


7. ARRANJO FÍSICO E FIXAÇÃO DA ELETRÔNICA (AVIONICS LAYOUT)

O posicionamento interno dos componentes (Topologia EMI e Centro de Gravidade) dita o sucesso e a ausência de ruídos no controle do VANT. Cada milímetro importa.

7.1. Bloco de Propulsão (Frontal)

Motor A2212: Afixado na Parede de Fogo (4x4cm) na ponta extrema do novo nariz (+10 cm à frente do Bordo de Ataque). Regra de Ouro: Colocar arruelas no suporte do motor para forçar o Down-Thrust (2 a 3 graus para baixo) e o Right-Thrust (1.5 graus para a direita).

Bateria LG HG2 2S: Posicionada no compartimento interno do nariz. Seu centro de massa (metade do comprimento da bateria) deve ficar a exatos -5,6 cm (ou seja, 5,6 cm para a frente do Bordo de Ataque). Fixação obrigatória com fita velcro ou espuma para amortecer solavancos (Baterias de lítio soltas se comportam como pêndulos e desestabilizam o voo).

ESC 30A: Deve ficar a 0 cm (exatamente na linha do Bordo de Ataque). Obrigatório: Faça recortes ou janelas no XPS para que o ESC receba fluxo de ar (arrefecimento). Amarre com abraçadeira plástica (enforca-gato). Os cabos de força entre o ESC e a bateria devem ser os mais curtos possíveis para evitar pico indutivo que possa "queimar" ou resetar o ESP32.

7.2. Bloco Lógico e Navegação (Centro e Dorsal)

IMU (Sensor MPU6050): O componente mais crítico da aeronave. Deve ser posicionado exatamente sobre o novo CG (5.8 a 6.3 cm do Bordo de Ataque), perfeitamente nivelado com o plano da asa e apontando perfeitamente para a frente. Fixação: Colado com fita dupla-face 3M de espuma firme. Nunca use cola quente, pois ela endurece como pedra e transfere toda a micro-vibração da hélice para o giroscópio, "enlouquecendo" o cálculo de PID.

Controladora (ESP32): Posicionada imediatamente à frente ou ao lado do MPU6050 (entre 2 e 5 cm do Bordo de Ataque). Fixação em base de plástico com amortecimento leve para evitar ruptura de soldas.

GPS (NEO-6M): Fixado no dorso (parte superior/externa) do Pod Central. O céu deve estar livre para a antena de cerâmica. Para evitar ruído de alta potência, ele deve ficar a pelo menos 8 centímetros de distância do ESC.

7.3. Bloco de Telemetria (Traseira)

Rádio SX1278 (LoRa 433MHz): Posicionado mais ao fundo do Pod (a 10 ou 12 cm do Bordo de Ataque).

Antena de Rádio: A antena pigtail/chicote deve preferencialmente ficar em posição Vertical (saindo por um furo pelo teto do Pod ou pendurada para baixo) para coincidir com a polarização da antena do rádio da sua estação-base, garantindo a performance teórica de 10 Km de distância.

Servo do Profundor (Pitch): Embutido no final da fuselagem (onde antes ficava o motor antigo). Fixado firmemente com epóxi ao XPS. O Horn (braço do servo) deve movimentar livremente o arame de aço conectado à cauda através das guias ao longo da haste de fibra.

8. DINÂMICA DE CONTROLE E INTEGRAÇÃO DE AUTOPILOT (ESP32 / INAV)

A alteração do layout Flying Wing para Tractor Convencional exige uma reconfiguração profunda do firmware de estabilização.

8.1. Matriz de Mixagem (Mixer / Servo Mapping)

A primeira regra é desativar a mixagem Delta/Flying Wing. Implementamos a mixagem diferencial nos ailerons para mitigar o arrasto de guinada adverso (adverse yaw) induzido pelo enflexamento de 12º em curvas coordenadas sem leme ativo.

Motor (CH1): Esc direto ao PWM.

Aileron Esquerdo (CH2): Responde à rolagem (Roll), com deflexão atenuada em 35% quando atua no sentido descendente (Differential ratio de 1.5:1).

Aileron Direito (CH3): Responde à rolagem (Roll), com deflexão atenuada em 35% quando atua no sentido descendente (Differential ratio de 1.5:1).

Profundor (CH4): O novo servo traseiro responde exclusivamente à arfagem (Pitch).

8.2. Autoridade Aerodinâmica e Limites Mecânicos (Throws)

Com um Braço de Alavanca ($L_h$) de 43,3 cm, sua cauda tem uma autoridade de torque imensa.

Ajuste Prático: Limite a deflexão mecânica máxima do profundor para no máximo 12 a 15 graus. Mais do que isso causará estol prematuro de cauda e looping involuntário.

8.3. Ajuste Fino de PID

Ganhos de PITCH (Redução Crítica): Devido à nova força de alavanca, se você usar os mesmos ganhos de Pitch da asa voadora antiga, o avião terá oscilações verticais severas. Reduza o ganho P do eixo Pitch em 30% a 40%.

Ganhos de ROLL (Leve Aumento): Como as winglets gigantes sumiram, o avião rolará com menos resistência (Menor Amortecimento de Rolagem). O ganho D (Derivativo) no eixo Roll deve ser sutilmente aumentado.

8.4. Navegação por "Bank and Yank" (Turn Coordination)

Sem Leme (Rudder) ativo para coordenar curvas, a controladora deve curvar usando Bank and Yank (Inclina a asa e puxa o profundor). No firmware, configure a dinâmica de voo como "Airplane sem leme". O sistema irá automaticamente comandar rolagem seguida de leve incremento de Pitch (up-elevator) durante a navegação por Waypoints.