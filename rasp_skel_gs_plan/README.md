# Percepção das Pistas de Trânsito, Definição do Goal State e Planejamento de Movimentação

Nesse diretório está o código que foi utilizado pela Raspberry para receber o occupancy grid, fazer a esqueletonização, 
definir um goal state e planejar a movimentação.

Os principais arquivos desta pasta são o [**`skel_gs_plan.cpp`**](./skel_gs_plan.cpp) e o [**`skel_gs_plan.h`**](./skel_gs_plan.h). 
Nesses dois arquivos está o código necessário para receber o occupancy grid até gerar uma trajetória.

Na pasta [**`skel_gs_results`**](./skel_gs_results) são salvos alguns resultados intermediários da esqueletonização e da definição do goal state.
Já na pasta [**`solution_paths`**](./solution_paths) são salvas as trajetórias geradas pelo planejamento de movimentação.

Para visualizar os resultados presentes em `skel_gs_results` e `solution_paths` os arquivos 
[**`read_skel_gs_results_and_plot.py`**](./read_skel_gs_results_and_plot.py) e 
[**`read_plan_results_and_plot.py`**](./read_plan_results_and_plot.py) podem ser executados. 
As imagens geradas serão salvas nas pastas 
[**`visualize_skel_gs_results`**](./visualize_skel_gs_results) e [**`visualize_plan_results`**](./visualize_plan_results).

Deixamos nas pastas `skel_gs_results`, `solution_paths`, `visualize_skel_gs_results` e `visualize_plan_results` alguns exemplos de resultados.

Para realizar algum teste sem a Nano presente, podemos simular seu comportamento usando o arquivo 
[**`client_send_ogs_checkpoints.cpp`**](./client_send_ogs_checkpoints.cpp). Esse arquivo envia occupancy grids da mesma forma que a Nano os envia, 
a diferença é que esse arquivo lê occupancy grids já gerados (não os gera). O tempo de espera entre envios de occupancy grids pode ser mudado no código. 
Esse tempo simula o tempo que a Nano levaria para fazer a segmentação semântica, gerar a máscara e o occupancy grid, dentre outros pequenos detalhes.
Além disso, neste arquivo é possível adicionar checkpoints em posições do occupancy grid para testar o comportamento de todo o processo quando o Crawler
estiver próximo de um checkpoint.

Os occupancy grids utilizados e seus checkpoints (se desejar adicioná-los) estão definidos em 
[**`checkpoint_file_list/list.txt`**](./checkpoint_file_list/list.txt). A maneira de definir occupancy grids e checkpoints é bem intuitiva.

> Os occupancy grids devem estar armazenados na pasta 
> [**`../occupancy_grid_tests_p/correct_nano_ogs/new_ogs/`**](../occupancy_grid_tests_p/correct_nano_ogs/new_ogs)

Por fim, os arquivos [**`exp_read_skel_gs_results_and_plot.py`**](./exp_read_skel_gs_results_and_plot.py) e 
[**`exp_read_plan_results_and_plot.py`**](./exp_read_plan_results_and_plot.py) podem ser usados para visualizar os resultados equivalentes aos 
armazenados nas pastas `skel_gs_results` e `solution_paths` se os resultados estiverem armazenados em 
`../exps/<exp_number>/skel_gs_results` e `../exps/<exp_number>/solution_paths`. As imagens serão salvas em
`../exps/<exp_number>/visualize_skel_gs_results` e `../exps/<exp_number>/visualize_solution_paths`. Para escolher o exp_number 
basta passar o número como entrada para os .py.
