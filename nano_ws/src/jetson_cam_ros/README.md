# ROS Package da Segmentação Semântica e Geração do Occupancy Grid

Os arquivos [**`CMakeLists.txt`**](./CMakeLists.txt) e [**`package.xml`**](./package.xml) são os arquivos necessários para rodar 
colcon build para este pacote.

O arquivo [**`src/segnet_intel02.cpp`**](./src/segnet_intel02.cpp) possui o código para fazer toda a segmentação semântica e geração do occupancy grid.
Esse código, inclusive salva as imagens capturas, as máscaras e os occupancy grids gerados. Além disso, ele envia o occupancy grid para a Raspberry.
Esse é o arquivo que será o nodo ROS deste pacote.

Já o arquivo [**`src/simulate_capture_image.cpp`**](./src/simulate_capture_image.cpp) simula que está capturando imagens, 
realizando a segmentação semântica e gerando os occupancy grids. Esse código lê occupancy grids salvos e os envia para a Raspberry.
O arquivo [**`src/get_file_names.py`**](./src/get_file_names.py) gera uma lista de occupancy grids salvos para serem lidos pelo
`src/simulate_capture_image.cpp`. Esses arquivos não possuem relação com ROS.
