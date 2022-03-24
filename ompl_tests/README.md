# Experimentos com o Planejamento de Movimentação

Nesse diretório está o código que foi utilizado para realizar experimentos com o planejamento a movimentação.
Ressaltamos que utilizamos a OMPL para realizar o planejamento de movimentação.

Os principais arquivos deste diretório estão na pasta [**`tests`**](./tests/). 

Os arquivos [**`benchmark_results_geometric.py`**](./benchmark_results_geometric.py) e [**`benchmark_results_control.py`**](./benchmark_results_control.py) servem para obter resultados 
dos experimentos realizados com planners geométricos e planners baseados em controle respectivamente. Algumas funções utilizadas nesses arquivos
estão implementadas em [**`analyse_results`**](./analyse_results/).

Já o arquivo [**`visualize_solution_all.py`**](./visualize_solution_all.py) cria imagens dos caminhos/trajetórias salvos em 
[**`solution_paths`**](./solution_paths/) e as armazena em [**`visualize_solutions`**](./visualize_solutions/).

Os experimentos presentes em `tests` geram logs que são salvos em [**`logs`**](./logs/) e um banco de dados que fica
salvo em [**`databases`**](./databases/).

O arquivo [**`files.txt`**](./files.txt) indica quais occupancy grids devem ser lidos.

Deixamos algumas anotações sobre alguns componentes e parâmetros de planners da OMPL no arquivo [**`lembretes_params.txt`**](./lembretes_params.txt).

A pasta [**`tables`**](./tables/) serve apenas para gerar alguns resultados em latex.

Deixamos alguns exemplos de resultados esperados em `logs`, `databases`, `solution_paths` e `visualize_solutions`. 
