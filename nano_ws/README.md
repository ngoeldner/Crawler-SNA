# Segmentação Semântica e Geração do Occupancy Grid

Nesse diretório está o código que foi utilizado pela Nano para capturar imagens, realizar a segmentação semântica, 
gerar o occupancy grid e enviá-lo à Raspberry.

## Funcionamento
A rede neural convolucional usada para realizar a segmentação semântica é uma HRNet. O treinamento da rede foi feito utilizando o método fit\_one\_cycle da biblioteca FastAI. O dataset utilizado foi o RTK Dataset. Nesse dataset a segmentação semântica possui 12 classes, sendo estrada asfaltada, pavimentada e de terra, e falhas na pista algumas dessas classes.

Para integrar a captura das imagens com o processamento feito pela HRNet, utilizamos uma biblioteca da Nvidia chamada jetson-inference. Essa biblioteca faz com que a rede neural seja processada pela GPU da Nano acelerando o processamento das imagens e fazendo com que cada imagem leve entre 95 e 160 milissegundos (a maioria dos valores ficam bem próximos de 160 e uma outra grande parte dos valores ficam próximos de 95) para obtermos a saída da HRNet (que ainda não é a imagem segmentada e sim a "probabilidade" de cada classe para cada pixel). Porém, infelizmente, a geração da imagem segmentada é feita fora da GPU causando um acréscimo de aproximadamente 195 milissegundos. Esse pós-processamento poderia ser acelerado pela GPU, entretanto teríamos que fazer algumas modificações na biblioteca jetson-inference. Em cada iteração fazemos mais do que apenas obter a imagem segmentada, salvamos a imagem original e a segmentada; geramos, enviamos e salvamos o occupancy grid; além de mais alguns pequenos detalhes. Em geral essas operações a mais demandam por volta de 100 milissegundos, totalizando, em geral, aproximadamente 450 milissegundos por iteração. Devemos notar que esses valores são para imagens RGB 800x600. Outro detalhe é que a imagem segmentada gerada possui 400x300 pixels.

No caso do Crawler, optamos por usar a imagem segmentada para gerar um occupancy grid. Porém, como uma imagem (capturada pela câmera monocular) não nos fornece informações suficientes para representarmos um ambiente 3D, decidimos representar o ambiente como um occupancy grid 2D e assumimos que tudo visto na imagem está no nível da pista e a altura de tudo na imagem é zero.

O OG representa uma visão de cima da pista e perpendicular ao plano da pista. A área que decidimos representar no OG é dada por um retângulo com 10 metros de comprimento, 8 metros de largura, alinhado com o yaw do Crawler e inicia a 1 metro a partir da frente do Crawler. Certamente, esses valores não são os melhores, porém permitem a obtenção de bons resultados para o restante do SNA. Ademais, devemos notar que esses valores dependem muito da velocidade do Crawler, da câmera e da sua posição e orientação, das consequências ao tempo de processamento e à qualidade dos resultados de outras partes do SNA que dependem do OG, dentre diversos outros fatores. Fizemos diversos testes para chegarmos a esses valores e, frequentemente, fazemos modificações. Cada célula do OG possui 5 cm de comprimento e largura no mundo real, portanto o OG é uma matriz 200x160. Voltando o foco à criação do OG, para popularmos cada célula do OG com a informação de que se essa área é não navegável ou navegável, fazemos o seguinte: obtemos o centro de cada célula do OG em coordenadas 3D. Depois, projetamos cada um dos centros na imagem segmentada e obtemos coordenadas em pixels. Se o pixel está fora da imagem ou se a classe do pixel na imagem segmentada representa uma área não navegável, a respectiva célula do OG é considerada não navegável; caso contrário, a célula do OG é considerada navegável. 

Para fazer a projeção dos pontos, usamos o modelo de câmera com distorção de Brown-Conrady; sendo que os parâmetros intrínsecos da câmera foram estimados (esse processo é conhecido por calibração da câmera) com os componentes presentes em [**`camera_calibration`**](../camera_calibration) e os parâmetros extrínsecos foram obtidos da seguinte forma: medimos a altura da câmera no Crawler; admitimos que o yaw da câmera é o mesmo que o do Crawler, que o roll e o pitch não são alterados e que o roll é zero; o pitch estimamos usando marcações no piso e comparando a posição da marcação no mundo real com a marcação na projeção. Com o OG gerado, o enviamos por TCP via Ethernet para a Raspberry para que a percepção das pistas seja feita.

> O modelo de Brown-Conrady considera as distorções que uma imagem capturada pode apresentar. Devemos notar que as câmeras reais não são equivalentes ao modelo de câmera de pinhole, elas geram algumas diferenças na imagem que nesse caso consideramos distorções. Dependendo da intensidade dessas distorções, o occupancy grid pode ficar bem distante da realidade.


Exemplos de segmentação semântica e geração do OG:

<img src="../docs/images/final.png" alt="HTML5 Icon" style="width:50vw;display: block; margin-left: auto; margin-right:auto;">


## Arquivos
Os arquivos `classes.txt` `colors.txt` `hrnet_w18.onnx` são arquivos necessários para realizar a segmentação semântica. 
Em `classes.txt` estão os nomes das classes da rede; em `colors.txt` estão as cores em RGB de cada classe utilizadas para gerar a máscara
e `hrnet_w18.onnx` é a rede neural utilizada.

Em [**`src`**](./src) ficam os ROS packages da Nano. Neste caso, há apenas um que é o [**`jetson_cam_ros`**](./src/jetson_cam_ros).
