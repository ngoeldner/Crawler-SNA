# Testes para a geração do Occupancy Grid

Nesse diretório está o código de teste para a geração de occupancy grids.

Com [**`occupancy_grid_correct_nano_ogs_test.cpp`**](./occupancy_grid_correct_nano_ogs_test.cpp) é possível testar a geração de occupancy grids
usando a projeção inversa presente na biblioteca librealsense da intel. É possível ajustar os parâmetros intrínsecos e extrínsecos da câmera.
O modelo de distorção usado é o modelo de Brown-Conrady. Deve-se selecionar alguma imagem presente em `../correct_nano_ogs/imgs/` e
o resultado será salvo em `../correct_nano_ogs/new_ogs/`. 

[**`occupancy_grid_correct_nano_ogs_test_pitch.cpp`**](./occupancy_grid_correct_nano_ogs_test_pitch.cpp) serve para auxiliar o ajuste manual
do ângulo do pitch da câmera. Deve-se fazer uma marcação no chão e ajustar as faixas vermelhas para a sua posição real considerando o pitch desejado.
Deve-se selecionar alguma imagem presente em `../correct_nano_ogs/imgs/` e
o resultado será salvo em `../correct_nano_ogs/new_ogs/`.

Com [**`occupancy_grid_correct_nano_ogs.cpp`**](./occupancy_grid_correct_nano_ogs.cpp) é possível testar a geração de occupancy grids
usando a projeção inversa presente na biblioteca OpenCV. É possível ajustar os parâmetros intrínsecos e extrínsecos da câmera.
O modelo de distorção usado é o modelo de Brown-Conrady. Deve-se selecionar alguma imagem presente em `../correct_nano_ogs/imgs/` e
o resultado será salvo em `../correct_nano_ogs/new_ogs/`. Porém, ressaltamos que nos deparamos com alguns resultados não muito realistas 
utilizando a projeção inversa da OpenCV.

Por fim, [**`lembretes_imagens.txt`**](./lembretes_imagens.txt) é utilizado por outros programas para selecionar algumas imagens.
