# Módulos, ferramentas e experimentos do Crawler

O Crawler é um veículo autônomo em desenvolvimento. Abaixo há uma fotografia do Crawler.

<img src="https://github.com/ngoeldner/Crawler-SNA/raw/master/docs/images/fotografia_Crawler.jpg">

Os dispositivos presentes no Crawler estão representados na imgem abaixo.

<img src="https://github.com/ngoeldner/Crawler-SNA/raw/master/docs/images/dispositivos_e_conexoes.png">

O arquitetura do software do Crawler está representada na imagem abaixo.

<img src="https://github.com/ngoeldner/Crawler-SNA/raw/master/docs/images/software-org.png">

Neste repositório estão presentes os componentes da segmentação semântica e mapeamento em [**`nano_ws`**](./nano_ws/). 

Já a percepção de pistas, 
decisão de comportamento (definição do goal state) e planejamento de movimentação estão em [**`rasp_skel_gs_plan`**](./rasp_skel_gs_plan/). 

Há diversos experimentos que realizamos também. 
Em [**`skeletonize_and_goal_state_tests`**](./skeletonize_and_goal_state_tests/) estão os experimentos 
relacionados com a esqueletonização e definição do goal state.
Em [**`occupancy_grid_tests_p`**](./occupancy_grid_tests_p/) estão os experimentos com a geração do occupancy grid. 
Em [**`ompl_tests`**](./ompl_tests/) estão os experimentos com o planejamento de movimentação.

Na pasta [**`exps`**](./exps/), salvamos os nossos experimentos, aqui deixamos um experimento de exemplo.

Em [**`camera_calibration`**](./camera_calibration/) há o código necessário para encontrar os parâmetros intrínsecos de um câmera. 
