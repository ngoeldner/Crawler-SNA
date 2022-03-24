# Segmentação Semântica e Geração do Occupancy Grid

Nesse diretório está o código que foi utilizado pela Nano para capturar imagens, realizar a segmentação semântica, 
gerar o occupancy grid e enviá-lo à Raspberry.

Os arquivos `classes.txt` `colors.txt` `hrnet_w18.onnx` são arquivos necessários para realizar a segmentação semântica. 
Em `classes.txt` estão os nomes das classes da rede; em `colors.txt` estão as cores em RGB de cada classe utilizadas para gerar a máscara
e `hrnet_w18.onnx` é a rede neural utilizada.

Em [**`src`**](./src) ficam os ROS packages da Nano. Neste caso, há apenas um que é o [**`jetson_cam_ros`**](./src/jetson_cam_ros).
