# Experimentos com o Planejamento de Movimentação

Em [**`read_og.h`**](./read_og.h) estão algumas definições e funções usadas pelos .cpp.

O arquivo [**`benchmark_test5_rrtg_rrt_star.cpp`**](./benchmark_test5_rrtg_rrt_star.cpp) faz experimentos com algumas configurações do RRT geométrico e com o RRT*.

O arquivo [**`benchmark_test6_rrtc_sst.cpp`**](./benchmark_test6_rrtc_sst.cpp) faz experimentos com algumas configurações do RRT baseado em controle e com o SST.

O arquivo [**`benchmark_test7_sst_copy_bad_params.cpp`**](./benchmark_test7_sst_copy_bad_params.cpp) faz experimentos com uma configuração não tão boa de parâmetros para o planejamento e para o SST.

O arquivo [**`benchmark_test7_sst.cpp`**](./benchmark_test7_sst.cpp) faz experimentos com uma configuração boa de parâmetros para o planejamento e para o SST.

[**`get_info.py`**](./get_info.py) exibe algumas informações relacionadas com a distância que o planejamento conseguiu ficar da região do goal state. Essas informações 
foram salvas por [**`benchmark_test7_sst_copy_bad_params.cpp`**](./benchmark_test7_sst_copy_bad_params.cpp) e 
por [**`benchmark_test7_sst.cpp`**](./benchmark_test7_sst.cpp) em [**`dist_info.txt`**](./dist_info.txt)
