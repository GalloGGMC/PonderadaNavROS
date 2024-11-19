# Navegação Reativa com ROS

- Nome: Gabriel Gallo Menequini Coutinho

## Intruções de execução (Ubuntu 22.04.5 LTS):

- Clone o repositório localmente
- Faça a instalação do ROS2 Humble (tutorial: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Verifique a instalação do Python, abra um terminal e rode o comando `sudo apt install python3-pygame`
- Neste terminal, insira os comandos:
   ```
   cd cg
   colcon build
   source install/setup.bash
   ros2 run cg maze
   ```
  Será aberta uma guia com o labirinto
- Abra um novo terminal na mesma pasta e insira os comandos:
  ```
  source install/setup.bash
  cd ..
  cd Solver
  colcon build
  source install/setup.bash
  ros2 run maze_solver solve
  ```
OBS: o pacote cg deve ser compilado antes pois o pacote Solver depende do cg_interfaces
