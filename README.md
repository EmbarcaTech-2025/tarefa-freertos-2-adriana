
# Tarefa: Roteiro de FreeRTOS #2 - EmbarcaTech 2025

Autor: Adriana Rocha
       Arthur
       Carlos

Curso: Residência Tecnológica em Sistemas Embarcados

Instituição: EmbarcaTech - HBr

Campinas, 22 de junho de 2025


#Projeto: Simulação de Carro no Raspberry Pi Pico com FreeRTOS

###Imagine que queremos criar um pequeno "painel de controle" para um carro, mas em vez de um carro de verdade, vamos simular tudo em um pequeno computador chamado Raspberry Pi Pico. Este "painel" terá um joystick para controlar a aceleração e o freio, luzes (LEDs) para indicar o que está acontecendo (como frear ou acelerar), uma buzina, e até mesmo um som de motor que muda com a "velocidade". Todas essas informações serão mostradas em uma pequena tela.

O desafio é que tudo isso precisa acontecer ao mesmo tempo: ler o joystick, calcular a velocidade, acender as luzes, fazer o som, e atualizar a tela. É aqui que entra o FreeRTOS, um "sistema operacional" especial para pequenos computadores (microcontroladores), que nos ajuda a organizar tudo para que pareça que as coisas estão acontecendo paralelamente.

1. O Cérebro: Raspberry Pi Pico e o Chip RP2040
O coração do nosso projeto é o Raspberry Pi Pico. Pense nele como um pequeno computador muito especializado. Ele tem um chip principal chamado RP2040, que é um microcontrolador. Ao contrário de um computador comum (como seu PC ou notebook), que executa programas complexos e tem um sistema operacional como Windows ou Linux, o Pico é projetado para tarefas mais simples e dedicadas.
Por que o Pico é bom para isso? Ele é pequeno, barato, tem muitos pinos para conectar sensores e atuadores (como LEDs e motores), e é rápido o suficiente para nossas simulações.

2. O Maestro: FreeRTOS - O Sistema Operacional em Tempo Real
Aqui está um conceito chave: o FreeRTOS. Normalmente, um programa em um microcontrolador executa suas instruções uma após a outra. Se ele estiver lendo o joystick, não pode estar atualizando a tela ao mesmo tempo. Isso seria um problema para nosso carro, que precisa de tudo funcionando simultaneamente!

O FreeRTOS é um Sistema Operacional em Tempo Real (RTOS). Ele não é como o Windows, que tem uma interface gráfica. Ele é um "maestro" que gerencia as tarefas do nosso programa.
Tarefas (Tasks): No FreeRTOS, dividimos nosso programa em pedaços independentes chamados "tarefas". Por exemplo, teremos uma tarefa para ler o joystick, outra para calcular a física do carro, outra para controlar os LEDs, outra para fazer o som do motor e outra para atualizar a tela.

Multitarefa (ou Concorrência): O FreeRTOS faz com que pareça que todas essas tarefas estão executando ao mesmo tempo. Na realidade, o RP2040 só tem dois "cérebros" (chamados "cores" ou núcleos de processamento). O FreeRTOS alterna muito rapidamente entre as tarefas, dando um pouquinho de tempo de processamento para cada uma, tão rápido que para nós parece que estão todas rodando em paralelo.

Prioridades: Podemos dizer ao FreeRTOS quais tarefas são mais importantes. Por exemplo, a tarefa que lê o joystick pode ter uma prioridade mais alta, porque queremos que ela responda rapidamente aos nossos comandos.

Sincronização e Comunicação: O FreeRTOS também oferece ferramentas para as tarefas "conversarem" entre si de forma segura (para não bagunçar os dados) e se coordenarem. Usaremos principalmente Filas (Queues) para isso.

3. Conversando entre Tarefas: Filas (Queues)
Imagine que você tem várias pessoas trabalhando em uma linha de montagem, e uma pessoa precisa passar uma peça para a outra. Se elas jogarem a peça aleatoriamente, pode dar errado. Uma fila é como uma esteira rolante ou uma caixa organizada onde uma pessoa coloca a peça e a próxima pega quando estiver pronta.

No nosso projeto, as tarefas precisam compartilhar informações:
A "tarefa Joystick" lê a posição do joystick e os botões. Onde ela coloca essa informação para que outras tarefas a usem? Em uma fila!
A "tarefa Controle do Carro" calcula a velocidade, o RPM e a marcha. Onde ela coloca o status atual do carro para que a tela e o som do motor o usem? Em outra fila!
Usamos QueueHandle_t para criar e manipular essas filas no FreeRTOS.
xQueueCreate(): É a função que cria a fila. Dizemos a ela quantos "itens" a fila pode guardar e qual o tamanho de cada item.
xQueueSend() ou xQueueOverwrite(): A tarefa que gera a informação "coloca" o item na fila. xQueueOverwrite() é especial porque se a fila estiver cheia, ela joga fora o item mais antigo para colocar o novo, garantindo que sempre tenhamos os dados mais recentes. Isso é perfeito para dados que mudam constantemente, como a posição do joystick ou o status do carro.
xQueueReceive() ou xQueuePeek(): A tarefa que precisa da informação "pega" o item da fila. xQueueReceive() remove o item da fila. xQueuePeek() apenas "olha" o item sem removê-lo, o que é útil se várias tarefas precisam ver o mesmo dado.

4. Os Componentes de Hardware e Suas Funções
Vamos ver como os pedaços físicos se encaixam:

Joystick (Módulo KY-023):
Tem dois eixos analógicos (X e Y) e um botão (SW) embutido. Alguns modelos podem ter botões adicionais (A e B), que usaremos para o ABS e Airbag.
O eixo Y é o que nos interessa para controlar a aceleração/freio. Ele fornece uma tensão variável que o Pico lê usando o ADC (Conversor Analógico-Digital).

Os botões (SW, A, B) são lidos como entradas digitais.
LEDs RGB (Vermelho, Verde, Azul):
São pequenas luzes que acendemos e apagamos. Cada cor indica um estado diferente do carro (freio, aceleração, buzina).
Conectados a pinos GPIO (General Purpose Input/Output) do Pico. GPIOs são pinos que podemos configurar para "ligar" (HIGH) ou "desligar" (LOW) uma corrente elétrica.
Buzzer Passivo:

Um pequeno componente que emite som. Usaremos um para a buzina (ligando/desligando em um pino GPIO) e outro para o som do motor (usando PWM para variar a altura do som).
Display OLED SSD1306 (128x64):
Uma pequena tela preta e branca que exibe informações. Perfeita para mostrar velocidade, RPM e outros status do carro.

Comunica-se com o Pico usando um protocolo chamado I2C. I2C é como uma linguagem secreta que dois dispositivos (o Pico e o OLED) usam para trocar informações através de apenas dois fios.

5. O CMakeLists.txt - O "Arquiteto" do Projeto
Este arquivo é como a planta da sua casa. Ele não contém o código real, mas diz ao compilador (o programa que transforma seu código em algo que o Pico entende) como montar tudo.
cmake_minimum_required(VERSION 3.12): Garante que você esteja usando uma versão do CMake que entenda todas as instruções.
set(PICO_BOARD pico ...): Indica ao CMake que estamos construindo para um Raspberry Pi Pico.
set(PICO_SDK_PATH "C:/Users/user/.pico-sdk/sdk/2.1.1"): ATENÇÃO AQUI! Esta linha aponta para onde o "kit de desenvolvimento" (SDK) do Pico está no seu computador. Se você copiar este projeto, MUITO PROVAVELMENTE precisará mudar este caminho para onde o SDK está no SEU computador. Uma alternativa melhor é definir a variável de ambiente PICO_SDK_PATH no seu sistema operacional, e o CMake a encontrará automaticamente.

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake): Importa todas as ferramentas e funções que o Pico SDK oferece para o CMake.
if (DEFINED ENV{FREERTOS_KERNEL_PATH})...: Faz o mesmo para o FreeRTOS. Ele tenta encontrar o FreeRTOS através de uma variável de ambiente ou em uma pasta FreeRTOS dentro do seu projeto. É uma boa prática clonar o FreeRTOS-Kernel em uma pasta chamada FreeRTOS na raiz do seu projeto.
project(embarcatech-tarefa-freertos-2- C CXX ASM): Dá um nome ao seu projeto e diz que ele usa as linguagens C, C++ e Assembly.

pico_sdk_init(): Inicializa as configurações do SDK do Pico no CMake.
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake): Importa as configurações específicas do FreeRTOS para o chip RP2040 do Pico.
add_executable(...): Lista todos os arquivos .c que fazem parte do seu programa. O compilador vai juntar todos eles para criar o programa final. Se um arquivo .c não estiver aqui, ele não será compilado e sua funcionalidade não estará no programa.
pico_enable_stdio_usb(...): Habilita a comunicação serial via cabo USB do Pico. Isso permite que você veja mensagens de depuração (que escrevemos com printf()) no seu computador.
target_include_directories(...): Diz ao compilador onde encontrar os arquivos de cabeçalho (.h). Por exemplo, include, src, oled.
target_link_libraries(...): Lista as bibliotecas que seu código usa. As bibliotecas são conjuntos de funções pré-escritas que facilitam nossa vida. Por exemplo, hardware_i2c para falar com o OLED, hardware_adc para o joystick, hardware_pwm para o som do motor, e FreeRTOS-Kernel-Heap4 para o FreeRTOS.
pico_add_extra_outputs(...): Gera o arquivo .uf2, que é o formato fácil de arrastar e soltar para gravar o programa no Pico.

6. A Lógica em Ação: Como Cada Tarefa Funciona
Vamos ver o que cada arquivo .c faz e como as tarefas interagem.
main.c - O Ponto de Partida
Este é o primeiro código a ser executado quando o Pico liga. Ele é responsável por:
Preparar o Pico: stdio_init_all() configura a comunicação USB para que possamos ver mensagens no terminal do computador.

Inicializar o OLED: ssd1306_init() liga e configura a tela.
Criar as "Esteiras de Comunicação" (Filas):
xJoystickQueue = xQueueCreate(5, sizeof(joystick_data_t));: Cria uma fila para dados do joystick, capaz de armazenar 5 "pacotes" de dados do joystick.
xCarStatusQueue = xQueueCreate(5, sizeof(car_status_t));: Cria uma fila para o status do carro (velocidade, RPM, etc.), também com 5 pacotes.
configASSERT(...): São verificações importantes. Se, por algum motivo, a fila não puder ser criada (falta de memória, por exemplo), o programa irá parar aqui e nos avisar.

Lançar as "Pessoas Trabalhando" (Tarefas): xTaskCreate() cria cada uma das tarefas que definimos, dando um nome a elas, um espaço na memória (pilha) para trabalharem, e uma prioridade:
vJoystickTask (tskIDLE_PRIORITY + 3): Mais importante, precisa ser rápida para ler o joystick.
vCarControlTask (tskIDLE_PRIORITY + 2): A lógica do carro, depende da leitura do joystick.
vCarIndicatorsTask (tskIDLE_PRIORITY + 1): Liga os LEDs e buzina, depende do controle e joystick.
vOledTask (tskIDLE_PRIORITY + 1): Atualiza a tela, depende do status do carro. Perceba que ela pede um pouco mais de memória de pilha (+ 200) porque trabalhar com a tela pode exigir mais recursos temporários.
vMonitorJoystickTask (tskIDLE_PRIORITY): Uma tarefa de "espião" para depuração, com a menor prioridade.
vEngineSoundTask (tskIDLE_PRIORITY + 0): Gera o som do motor.

Entregar o Controle ao Maestro (Scheduler): vTaskStartScheduler() é a chamada que faz o FreeRTOS assumir o controle. A partir daqui, ele decide qual tarefa rodar em qual momento. A função main() nunca mais será executada após esta linha, a menos que o FreeRTOS pare por algum erro grave.
include/joystick_task.h e src/joystick_task.c - O Leitor do Controle
joystick_data_t: Esta é uma "caixa" de dados (struct) que criamos para organizar as informações do joystick:
y_axis: O valor do eixo Y, que vai de -2047 (todo para trás) a +2048 (todo para frente), sendo 0 no centro.
sw_state: Estado do botão SW (buzina), true se pressionado.
button_A_state: Estado do botão A (ABS), true se pressionado.
button_B_state: Estado do botão B (Airbag), true se pressionado.
vJoystickTask(void *pvParameters):
Configuração dos Pinos:
adc_init() e adc_gpio_init(JOYSTICK_Y_PIN): Preparam o pino GP26 para ler voltagens analógicas do eixo Y do joystick.
gpio_init(PIN), gpio_set_dir(PIN, GPIO_IN), gpio_pull_up(PIN): Configuram os pinos dos botões (GP22, GP5, GP6) como entradas e ativam um resistor interno chamado "pull-up". Isso faz com que o pino fique "alto" (ligado) por padrão. Quando você pressiona o botão, ele "aterriza" o pino (coloca em "baixo" ou desligado). Por isso, no código, usamos ! (negação) na leitura (!gpio_get(PIN)) para que true signifique "botão pressionado".

Leitura e Filtro: O adc_read() lê o valor do joystick. Usamos uma pequena "mágica" (raw_y = (prev_y * 3 + raw_y) / 4;) para suavizar as leituras. É um filtro simples para evitar que pequenas flutuações no sensor causem saltos bruscos na velocidade.

Envio para a Fila: A cada 50 milissegundos (pdMS_TO_TICKS(50)), a tarefa lê os dados, os coloca na estrutura joystick_data_t e os envia para a xJoystickQueue usando xQueueOverwrite(). Essa função é crucial: se a fila já tiver dados antigos, ela os joga fora para garantir que a tarefa de controle do carro sempre receba a leitura MAIS RECENTE do joystick.
vTaskDelayUntil(&xLastWakeTime, xFrequency): Isso faz com que a tarefa espere exatamente 50 milissegundos antes de rodar novamente. É muito importante para garantir que a leitura seja regular e não consuma todo o tempo do processador.

include/car_status_data.h e src/car_control_task.c - O Cérebro do Carro
car_status_t: Esta é a "caixa" de dados principal do nosso carro. Ela contém todas as informações 

sobre o estado atual do veículo que outras tarefas precisam saber:
current_speed_kmh: Velocidade atual em Km/h.
current_rpm: Rotações por minuto do motor.
current_gear: Marcha atual (0 para Neutro, 1 a 5).
abs_active: Se o ABS está atuando (freio).
airbag_deployed: Se o airbag foi acionado (uma vez acionado, fica assim).
horn_active: Se a buzina está ligada.
red_led_active: Se o LED vermelho de indicação (freio/ABS) deve estar ligado.
vCarControlTask(void *pvParameters):
Recebe Dados do Joystick: Primeiro, ela tenta pegar os dados mais recentes da xJoystickQueue usando xQueueReceive() com um timeout de 0.

Simulação de ABS: Se o botão A (ABS) estiver pressionado, a velocidade é instantaneamente zerada (current_speed_float = 0.0f;) e as flags abs_active e red_led_active são ativadas. Isso simula uma frenagem de emergência.
Aceleração, Freio e Arrasto:

Se o joystick Y for positivo (> NEUTRAL_THRESHOLD_JOY), o carro acelera gradualmente com base em ACCELERATION_RATE.
Se for negativo (< -NEUTRAL_THRESHOLD_JOY), ele freia com BRAKE_RATE.
Se estiver no centro, a velocidade diminui lentamente por "arrasto" (DRAG_RATE), simulando a fricção.

A velocidade é sempre limitada a 0 e MAX_SPEED_KMH.

Cálculo de Marcha: A marcha (calculated_gear) é determinada pela current_speed_kmh. Por exemplo, se a velocidade for até GEAR_1_MAX_SPEED (20 Km/h), é 1ª marcha; se for até GEAR_2_MAX_SPEED (40 Km/h), é 2ª marcha, e assim por diante.

Simulação de Troca de Marcha: Se a marcha muda, há um pequeno atraso (vTaskDelay(pdMS_TO_TICKS(100))) e o RPM é momentaneamente jogado para o mínimo. Isso simula o tempo de engate e a interrupção da potência. O status do carro é enviado imediatamente (xQueueOverwrite) para que a tela e o som do motor mostrem essa interrupção.

Cálculo de RPM: O RPM do motor é calculado com base na velocidade e na marcha atual (gear_ratio[]). Quanto maior a marcha e a velocidade, maior o RPM.

Airbag: Se o botão B (Airbag) for pressionado e o airbag ainda não tiver sido acionado (!airbag_was_deployed_once), a flag airbag_was_deployed_once é definida como true, e permanece assim.

Buzina: O estado da buzina é diretamente ligado ao botão SW do joystick (horn_active = received_joystick_data.sw_state;).

Envio de Status: Finalmente, todos os dados atualizados são colocados na estrutura current_car_status e enviados para a xCarStatusQueue usando xQueueOverwrite(). Isso garante que a tela e o som do motor sempre recebam o status mais recente do carro.
vTaskDelayUntil(): Garante que essa tarefa execute a cada 50 milissegundos.
include/car_indicators_task.h e src/car_indicators_task.c - As Luzes e a Buzina
vCarIndicatorsTask(void *pvParameters):
Configuração dos Pinos: Prepara os pinos GP13 (Vermelho), GP11 (Verde), GP12 (Azul) para os LEDs e GP21 para o buzzer, configurando-os como saídas.

Teste Inicial: Ao iniciar, a tarefa faz um pequeno "show" de luzes e som para que você possa verificar se tudo está conectado corretamente.

Recebimento de Dados: Usa xQueuePeek() para "espiar" os dados das filas xJoystickQueue e xCarStatusQueue. O Peek é importante porque ele LÊ os dados, mas NÃO os REMOVE da fila. Isso permite que a vCarControlTask e a vOledTask também usem esses mesmos dados.

Lógica de LEDs e Buzzer:
LED Vermelho: Acende se car_status.red_led_active for true (significando ABS ativo) OU se o joystick Y estiver abaixo de BRAKE_THRESHOLD (frenagem normal).
LED Verde: Acende se NÃO houver ABS ativo E o joystick Y estiver acima de ACCELERATION_THRESHOLD (aceleração).

LED Azul e Buzzer de Buzina: Acendem e tocam juntos quando o botão SW do joystick é pressionado.
vTaskDelayUntil(): A tarefa se repete a cada 20 milissegundos. Isso garante que as luzes e a buzina respondam rapidamente aos comandos.

include/engine_sound_task.h e src/engine_sound_task.c - O Ronco do Motor
vEngineSoundTask(void *pvParameters):
Configuração de PWM:
gpio_set_function(BUZZER_PWM_PIN, GPIO_FUNC_PWM): Diz ao pino GP10 para funcionar como uma saída PWM.
pwm_gpio_to_slice_num() e pwm_gpio_to_channel(): O Pico tem "slices" e "canais" de PWM. Estas funções encontram qual slice e canal o pino GP10 pertence.
pwm_set_wrap(slice_num, PWM_WRAP_VAL - 1): Configura o valor máximo do contador do PWM.
pwm_set_enabled(slice_num, true): Liga o gerador de PWM.
Leitura de RPM: Usa xQueuePeek() para ler o current_rpm da xCarStatusQueue.

Geração de Frequência:
float freq = rpm / 10.0f;: A frequência do som é diretamente proporcional ao RPM. Quanto maior o RPM, mais agudo o som do motor. O divisor 10.0f é um fator de ajuste para que a frequência fique numa faixa audível e que soe bem.

Cálculo do Divisor de Clock (clkdiv): Esta é a parte "matemática" do PWM. A frequência final do som PWM é o clock do sistema do Pico (125 MHz) dividido pelo clkdiv e pelo PWM_WRAP_VAL. A fórmula é rearranjada para calcular o clkdiv necessário para obter a freq desejada.

pwm_set_clkdiv(slice_num, clkdiv): Aplica o divisor de clock.
pwm_set_chan_level(slice_num, channel, PWM_WRAP_VAL / 2): Define o "ciclo de trabalho" (duty cycle) do PWM para 50%. Isso cria uma onda quadrada, que é boa para gerar sons simples em um buzzer.
Periodicidade: A tarefa se repete a cada 50 milissegundos (pdMS_TO_TICKS(50)), atualizando o som do motor.

include/oled_task.h e src/oled_task.c - A Tela de Informações
vOledTask(void *pvParameters):
Inicialização do OLED: ssd1306_init() já prepara o display.

Leitura de Status: Tenta receber o car_status_t mais recente da xCarStatusQueue usando xQueueReceive() com um timeout de xFrequency (100ms). Isso significa que a tarefa do OLED esperará por novos dados por até 100ms. Se não houver dados novos, ela espera um pouco antes de tentar de novo. Isso é eficiente, pois o display só precisa ser atualizado quando há uma mudança no status.

Limpar e Desenhar: Se novos dados são recebidos, a tela é primeiro limpa (ssd1306_clear()) para apagar o conteúdo antigo.

Formatação de Texto: Usa snprintf() para criar as frases como "Speed: 123 Km/h", "RPM: 05500", "Gear: N" (ou o número da marcha), "ABS: Active" ou "Inactive", e "Airbag: Deployed" ou "OK". O snprintf é seguro porque você especifica o tamanho máximo da string, evitando estouros de buffer.

Exibir no Display: ssd1306_draw_string() desenha essas frases no framebuffer do driver OLED.

Atualizar a Tela Física: O comando final é ssd1306_show(), que envia o conteúdo do framebuffer para o display OLED, tornando as informações visíveis.

vTaskDelayUntil(): Garante que a tarefa execute a cada 100 milissegundos, mantendo uma taxa de atualização consistente para o display.
oled/ssd1306.h e oled/ssd1306.c - O "Pintor" do OLED
Este é o driver que nos permite "desenhar" coisas no display OLED. Pense nele como uma ponte entre o software e o hardware do display.

ssd1306.h: Este arquivo é a "interface" do driver. Ele declara todas as funções que você pode usar para controlar o OLED (como ssd1306_init, ssd1306_clear, ssd1306_draw_string, ssd1306_show) e também define as constantes de comandos do chip SSD1306, que são usadas internamente pelo driver.

ssd1306.c: Esta é a "implementação" real das funções.
framebuffer: É um array de bytes (static uint8_t framebuffer[SSD1306_BUF_LEN];) que funciona como uma "tela de rascunho" na memória do Pico. Cada bit neste array representa um pixel no display. 

Quando você desenha um pixel, caractere ou string, você está, na verdade, mudando os bits neste framebuffer.

font8x8_basic: É uma tabela que armazena o desenho de cada caractere (letras, números, símbolos) em um formato de bitmap 8x8 pixels. Quando você chama ssd1306_draw_char(), o driver busca o desenho do caractere nesta tabela.

ssd1306_write_cmd() e ssd1306_write_data(): São funções internas que se comunicam diretamente com o chip SSD1306 via I2C. Elas enviam comandos (para configurar o display) ou dados de pixel (para desenhar na tela).

ssd1306_init(): Esta é a primeira função que você chama. Ela inicializa a comunicação I2C no Pico (configura os pinos GP14 e GP15 para I2C e define a velocidade de comunicação) e envia uma sequência de comandos ao chip SSD1306 para configurá-lo (ligar, definir contraste, rotação, etc.).

ssd1306_clear(): Simplesmente preenche todo o framebuffer com zeros, apagando todos os pixels da sua "tela de rascunho".

ssd1306_draw_pixel(x, y, on): Esta é a função mais básica de desenho. Ela calcula qual byte e qual bit no framebuffer correspondem ao pixel (x, y) e define esse bit como on (ligado) ou off (desligado).

ssd1306_draw_char(x, y, c) e ssd1306_draw_string(x, y, str): Usam ssd1306_draw_pixel() para desenhar caracteres e strings, respectivamente, usando a font8x8_basic.

ssd1306_show(): Esta é a função mais importante para a visualização! Após desenhar qualquer coisa no framebuffer (com clear, draw_pixel, draw_char, draw_string), você DEVE chamar ssd1306_show(). Ela lê o conteúdo do framebuffer e o envia para a memória de vídeo do display OLED, fazendo com que o que você desenhou apareça na tela física. Sem ela, nada aparece!
ssd1306_draw_line() e ssd1306_walk_horizontal_pixel(): São funções adicionais para desenhar linhas e uma animação de demonstração.

Conclusão Técnica e Lógica
A beleza deste projeto reside na forma como ele modulariza diferentes funcionalidades em tarefas FreeRTOS independentes. Cada tarefa é como um "mini-programa" rodando por si só, focado em uma única responsabilidade.
A vJoystickTask se preocupa apenas em ler o joystick e colocar os dados na fila.
A vCarControlTask se preocupa apenas em calcular a física do carro com base nos dados que ela pega da fila do joystick, e depois coloca o status atual do carro em outra fila.
A vCarIndicatorsTask e a vOledTask pegam os dados do status do carro da fila e se preocupam apenas em acender LEDs ou atualizar a tela.
A vEngineSoundTask pega o RPM do motor da fila e se preocupa apenas em fazer o som.
Essa separação de responsabilidades, combinada com a comunicação segura e eficiente das Filas (Queues) do FreeRTOS, evita que uma parte do código interfira na outra e torna o sistema muito mais robusto, fácil de entender, depurar e expandir. O FreeRTOS gerencia o tempo para que todas essas tarefas pareçam estar acontecendo simultaneamente, dando ao usuário uma experiência de simulação responsiva e dinâmica.
Ao entender como cada componente de hardware é interligado aos pinos específicos do Pico, como o CMakeLists.txt orquestra a compilação de tudo, e como cada tarefa do FreeRTOS contribui com sua lógica específica, você tem uma visão completa do projeto, desde os fundamentos eletrônicos até a arquitetura de software embarcado multitarefa.

__________________________________________________________________________________________________

Baixar o Pico SDK
Nota: O CMakeLists.txt deste projeto espera o SDK na versão 2.1.1 e um caminho específico C:/Users/user/.pico-sdk/sdk/2.1.1. Você precisará ajustar o PICO_SDK_PATH no CMakeLists.txt para o local onde você clonou o SDK ou criar um link simbólico se preferir manter o caminho original para compatibilidade com a extensão VS Code. Para o caminho C:/Users/user/.pico-sdk/sdk/2.1.1, se você usa Linux/WSL, pode ser algo como /mnt/c/Users/user/.pico-sdk/sdk/2.1.1.

3. Baixar o FreeRTOS Kernel
Para este projeto, o CMakeLists.txt procura o FreeRTOS em um subdiretório FreeRTOS ou via variável de ambiente FREERTOS_KERNEL_PATH.
Certifique-se de que a versão do FreeRTOS-Kernel seja compatível com o port RP2040. A estrutura esperada para importação no CMake é /portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake.

4. Definir Variáveis de Ambiente (Opcional, mas recomendado)
É uma boa prática definir a variável de ambiente PICO_SDK_PATH para o diretório do seu Pico SDK:
E se você não clonou o FreeRTOS dentro da pasta do projeto, defina FREERTOS_KERNEL_PATH:
Compilação e Upload

Navegue até o diretório raiz do projeto:

Crie um diretório build e navegue até ele:

Execute o CMake para configurar o projeto:

Se você usa a extensão VS Code para Pico, basta abrir o projeto no VS Code e ele deve configurar automaticamente.

Compile o projeto:

Isso gerará o arquivo .uf2 na pasta build, que é o firmware para o Pico.

Carregue o firmware para o Raspberry Pi Pico:

Pressione e segure o botão BOOTSEL no Pico.
Conecte o Pico ao seu computador via USB.

Solte o botão BOOTSEL. O Pico aparecerá como um dispositivo de armazenamento USB (unidade RPI-RP2).
Arraste o arquivo embarcatech-tarefa-freertos-2-.uf2 (encontrado em build/) para a unidade RPI-RP2.
O Pico irá reiniciar automaticamente e começar a executar o firmware.

Uso do Sistema
Após o upload do firmware:
Display OLED: Você verá a mensagem "Sistema Iniciado" e, em seguida, as informações de velocidade, RPM, marcha, ABS e Airbag serão exibidas e atualizadas em tempo real.

Joystick:
Mova o eixo Y para cima para acelerar e para baixo para frear.
Pressione o botão 'A' (associado ao BUTTON_A_PIN) para ativar o ABS e parar o carro instantaneamente.
Pressione o botão 'B' (associado ao BUTTON_B_PIN) para simular o acionamento do airbag (uma única vez).
Pressione o botão 'SW' (associado ao JOYSTICK_SW_PIN) para acionar a buzina.
LEDs e Buzzer: Observe os LEDs e o buzzer de buzina acenderem/acionarem de acordo com suas ações no joystick.

Som do Motor: O buzzer conectado ao PWM emitirá um som que varia em frequência com o RPM simulado do motor.
Console Serial: Conecte-se à porta serial USB do Raspberry Pi Pico (usando um terminal como Putty, CoolTerm, ou o terminal serial do VS Code) para ver as mensagens de depuração de cada tarefa, incluindo leituras do joystick e status do carro.

Estrutura do Projeto

Contribuições
Sinta-se à vontade para abrir issues, enviar pull requests ou sugerir melhorias.

main.c
car_control_task.c
car_indicators_task.c
engine_sound_task.c
joystick_task.c
oled_task.c
oled/ssd1306.c
include/car_control_task.h
include/car_indicators_task.h
include/car_status_data.h
include/engine_sound_task.h
include/FreeRTOSConfig.h
include/joystick_task.h
include/oled_task.h
oled/ssd1306.h

__________________________________________________________________________________________________

Glossário
ADC (Analog-to-Digital Converter): Conversor Analógico-Digital. Um componente que converte um sinal analógico (como a tensão de um joystick) em um valor digital que o microcontrolador pode processar.

API (Application Programming Interface): Conjunto de definições e protocolos que permitem que softwares se comuniquem uns com os outros. No FreeRTOS, são as funções como xTaskCreate(), xQueueSend(), etc.

CMake: Uma ferramenta de código aberto usada para gerenciar o processo de compilação de software usando uma abordagem independente de plataforma. Gera makefiles ou outros arquivos de projeto.

Duty Cycle: Em PWM, é a proporção do tempo em que um sinal está "ligado" (HIGH) em relação ao período total do sinal. Expresso em porcentagem.

Embedded System (Sistema Embarcado): Um sistema computacional com uma função dedicada dentro de um sistema mecânico ou elétrico maior. Projetado para uma tarefa específica.

extern: Palavra-chave em C que declara que uma variável ou função é definida em outro arquivo fonte, permitindo que ela seja usada no arquivo atual.

Framebuffer: Uma área da memória que contém uma representação em pixels do que deve ser exibido na tela. O driver SSD1306 escreve neste buffer e depois o transfere para o display.

FreeRTOS: Um sistema operacional em tempo real (RTOS) de código aberto para microcontroladores. Ele gerencia as tarefas, filas, semáforos e outros recursos de tempo real.

GPIO (General Purpose Input/Output): Pinos de entrada/saída de uso geral no microcontrolador que podem ser configurados como entradas ou saídas digitais, ou para funções especiais (I2C, PWM, ADC).

Heap: Uma área de memória onde programas podem alocar memória dinamicamente em tempo de execução (ex: com malloc() ou xTaskCreate() do FreeRTOS).

I2C (Inter-Integrated Circuit): Um protocolo de comunicação serial de dois fios (SDA e SCL) amplamente usado para comunicação de curta distância entre componentes, como microcontroladores e displays OLED.

Kernel: O núcleo de um sistema operacional que gerencia os recursos do sistema e as interações entre hardware e software. No FreeRTOS, é o responsável pelo agendamento das tarefas.

Mutex (Mutual Exclusion): Um objeto de sincronização usado para proteger recursos compartilhados, garantindo que apenas uma tarefa por vez possa acessá-lo.

Pico SDK (Software Development Kit): O kit de desenvolvimento de software oficial da Raspberry Pi Foundation para o Raspberry Pi Pico. Fornece bibliotecas e ferramentas para programar o RP2040.

PIO (Programmable I/O): Um subsistema no RP2040 que permite aos desenvolvedores definir interfaces de hardware personalizadas programando pequenos "state machines".

PWM (Pulse Width Modulation): Modulação por Largura de Pulso. Uma técnica para controlar a quantidade de energia entregue a uma carga, variando a largura de um pulso digital. Usada para controle de brilho de LEDs, velocidade de motores e geração de áudio.

Queue (Fila): Um mecanismo de comunicação entre tarefas no FreeRTOS que permite a troca segura de dados. Os dados são enviados para o final da fila e lidos do início (FIFO - First In, First Out).
Raspberry Pi Pico: Uma placa de microcontrolador pequena, rápida e versátil construída no chip RP2040 da Raspberry Pi.

README.md: Um arquivo de texto comum em projetos de software que fornece uma visão geral do projeto, instruções de build, uso e outras informações importantes.

RTOS (Real-Time Operating System): Sistema Operacional em Tempo Real. Um sistema operacional que garante que certas operações serão executadas dentro de prazos definidos.

Scheduler (Agendador): A parte do kernel do RTOS que decide qual tarefa deve ser executada a qualquer momento, com base em suas prioridades e outros critérios.

Semáforo: Um objeto de sincronização que pode ser usado para controlar o acesso a recursos ou para sinalizar a ocorrência de eventos entre tarefas.

SSD1306: Um chip controlador amplamente utilizado em pequenos displays OLED monocromáticos.

Stack (Pilha): Uma área de memória reservada para cada tarefa onde variáveis locais, parâmetros de função e endereços de retorno são armazenados temporariamente.

static: Palavra-chave em C que, quando aplicada a variáveis globais ou em escopo de arquivo, garante que elas sejam visíveis apenas dentro do arquivo em que foram definidas. Quando aplicada a variáveis locais dentro de uma função, garante que a variável preserve seu valor entre chamadas da função.

Task (Tarefa): No FreeRTOS, uma função independente que pode ser executada em concorrência com outras tarefas. Cada tarefa tem sua própria pilha e prioridade.

Tick: Uma interrupção periódica gerada por um timer do sistema, usada pelo FreeRTOS para manter o controle do tempo e para o agendamento de tarefas.

Time Slicing (Fatiamento de Tempo): Um modo de operação do scheduler onde tarefas de mesma prioridade compartilham o tempo da CPU, cada uma recebendo uma "fatia" de tempo para executar antes que o scheduler mude para a próxima.

vTaskDelay() / vTaskDelayUntil(): Funções do FreeRTOS para atrasar (suspender) a execução de uma tarefa por um período de tempo especificado. vTaskDelayUntil é preferível para atrasos periódicos precisos.

xQueueOverwrite() / xQueuePeek() / xQueueReceive(): Funções do FreeRTOS para interagir com filas. Overwrite substitui o item mais antigo se a fila estiver cheia. Peek lê sem remover. Receive lê e remove.

____________________________________________________________________________________________________
5. Referências
Documentação Oficial do Raspberry Pi Pico SDK:

Documentação Oficial do FreeRTOS:

Para o Display OLED SSD1306:

(Importante para entender os comandos e a arquitetura do display).
Recursos Adicionais para Raspberry Pi Pico e FreeRTOS:

(Exemplos de código que demonstram o uso de periféricos e FreeRTOS no Pico).
(Lista curada de recursos para o Pico).

____________________________________________________________________________________________________




## 📜 Licença
GNU GPL-3.0.
