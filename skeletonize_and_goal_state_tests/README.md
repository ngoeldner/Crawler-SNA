# Experimentos com a Esqueletonização e Definição do Goal State

Nesse diretório estão códigos que foram utilizados para realizar experimentos com a esqueletonização e com a definição do goal state.

[**`skeletonize_tests_time_and_good_plot.py`**](./skeletonize_tests_time_and_good_plot.py) realiza alguns 
testes relacionados com os diferentes métodos de esqueletonização.

[**`skeletonize_and_define_gs_all.py`**](./skeletonize_and_define_gs_all.py) 
mostra o passo a passo para todos occupancy grids presentes em 
[**`../occupancy_grid_tests_p/correct_nano_ogs/new_ogs/`**](../occupancy_grid_tests_p/correct_nano_ogs/new_ogs/). 
Esse código também exibe algumas informações relacionadas com o tempo de execução. As imagens geradas são salvas em 
[**`skeleton_to_goal_state`**](./skeleton_to_goal_state). Algumas informações são salvas na pasta [**`to_mp`**](./to_mp/) para os experimentos em
[**`../ompl_tests/tests`**](../ompl_tests/tests/)
usarem essas informações para realizarem o planejamento de movimentação. 


Os arquivos [**`skeletonize_and_send_skeleton.py`**](./skeletonize_and_send_skeleton.py) e
[**`read_skeleton_and_define_gs_all.cpp`**](./read_skeleton_and_define_gs_all.cpp) funcionam em conjunto para realizar alguns testes relacionados
com o tempo de execução. Nesse experimento o .py realiza a esqueletonização e obtém algumas informações do OG e as envia para o .cpp, 
que define o goal state. Eles trocam informações através de arquivos na pasta [**`to_c`**](./to_c/).

O arquivo [**`skeletonize_and_define_gs.h`**](./skeletonize_and_define_gs.h) possui funcões 
que são utilizadas por todos os .cpp que serão mencionados a seguir.

[**`read_og_info_and_send.py`**](./read_og_info_and_send.py) gera uma lista de occupancy grids para serem lidos. 
Os occupancy grids são selecionados inicialmente em
[**`../occupancy_grid_tests_p/occupancy_grid_tests/lembretes_imagens.txt`**](../occupancy_grid_tests_p/occupancy_grid_tests/lembretes_imagens.txt). 
[**`skeletonize_and_define_gs_all_opt.cpp`**](./skeletonize_and_define_gs_all_opt.cpp)
faz a esqueletonização e a definição do goal state dos occupancy grids selecionados por `read_og_info_and_send.py`.

[**`client_send_ogs_checkpoints.cpp`**](./client_send_ogs_checkpoints.cpp) e 
[**`skeletonize_and_define_gs_socket.cpp`**](./skeletonize_and_define_gs_socket.cpp) funcionam em conjunto. 
O `client_send_ogs_checkpoints.cpp` simula as
tarefas da Nano e envia o occupancy grid para `skeletonize_and_define_gs_socket.cpp` que recebe o occupancy, faz a esqueletonização e define o
goal state.

[**`skeletonize_and_define_gs_socket_threads.cpp`**](./skeletonize_and_define_gs_socket_threads.cpp) faz a mesma 
atividade que o `skeletonize_and_define_gs_socket.cpp`. Porém, 
`skeletonize_and_define_gs_socket_threads.cpp` possui threads para realizar auxiliar com a sincronizacão do que está sendo feito na Nano 
(ou no código que simula a Nano) e da esqueletonização e definição do goal state.

[**`read_results_and_plot.py`**](./read_results_and_plot.py) serve para gerar imagens dos 
resultados da execução dos arquivos `skeletonize_and_define_gs_all_opt.cpp` e `skeletonize_and_define_gs_socket.cpp`. As imagens são salvas 
em [**`visualize_cpp_results`**](./visualize_cpp_results/).

Em [**`checkpoint_file_list`**](./checkpoint_file_list/) há uma lista de checkpoints associados com occupancy grids para serem lidos.

Em [**`cpp_results`**](./cpp_results/), os .cpp que usam `skeletonize_and_define_gs.h` salvam seus resultados nessa pasta.

> Informações do funcionamento da esqueletonização e definição de goal states no Crawler podem ser encontradas em [**`rasp_skel_gs_plan`**](../rasp_skel_gs_plan/).