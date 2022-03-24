# Módulos, ferramentas e experimentos do Crawler

O Crawler é um veículo autônomo em desenvolvimento. Abaixo há duas fotografias do Crawler.

<img src="https://github.com/ngoeldner/Crawler-SNA/raw/master/docs/images/fotografia_crawler01.jpeg" alt="drawing" width="400">
<img src="https://github.com/ngoeldner/Crawler-SNA/raw/master/docs/images/fotografia_crawler02.jpeg" alt="drawing" width="250">


Os dispositivos presentes no Crawler estão representados na imagem abaixo.

<img src="https://github.com/ngoeldner/Crawler-SNA/raw/master/docs/images/dispositivos_e_conexoes.png" alt="drawing" width="400">

A arquitetura de software do Crawler está representada na imagem abaixo.

<img src="https://github.com/ngoeldner/Crawler-SNA/raw/master/docs/images/software-org.png" alt="drawing" width="400">

Neste repositório estão presentes os componentes da segmentação semântica e mapeamento em [**`nano_ws`**](./nano_ws/). 

Já a percepção de pistas, 
decisão de comportamento (definição do goal state) e planejamento de movimentação estão em [**`rasp_skel_gs_plan`**](./rasp_skel_gs_plan/). 

Há diversos experimentos que realizamos também. 
Em [**`skeletonize_and_goal_state_tests`**](./skeletonize_and_goal_state_tests/) estão os experimentos 
relacionados com a esqueletonização e definição do goal state.
Em [**`occupancy_grid_tests_p`**](./occupancy_grid_tests_p/) estão os experimentos relacionados com a geração do occupancy grid. 
Em [**`ompl_tests`**](./ompl_tests/) estão os experimentos relacionados com o planejamento de movimentação.

Na pasta [**`exps`**](./exps/), salvamos os nossos experimentos. Deixamos aqui um experimento de exemplo.

Em [**`camera_calibration`**](./camera_calibration/) há o código necessário para encontrar os parâmetros intrínsecos de uma câmera. 
