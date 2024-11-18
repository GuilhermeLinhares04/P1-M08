# Navegação com ROS

Este projeto engloba dois métodos distintos para navegação autônoma utilizando o ROS (Robot Operating System), com o objetivo de atingir um ponto específico de um local pré-determinado. O primeiro deles é a navegação reativa, na qual o algoritmo navega utilizando informações detectadas por sensores que indicam se há possibilidade de movimento ou não. O segundo é a navegação por mapa, que, como o nome indica, realiza um mapeamento do ambiente para que o algoritmo possa estabelecer sua rota antes mesmo de começar a se movimentar.

# Instalação e utilização

Para que a instalação seja realizada com êxito, é recomendado o uso do **Ubuntu 22.04**, com o  **ROS2** instalado e o **pygame** disponível. Abaixo segue o passo a passo ordenado para a instalação: 

1. Clone o repositório pela página principal do mesmo
2. Clone o repositório _https://github.com/rmnicola/Culling_Games_
3. Acesse o terminal do Ubuntu e entre na pasta _**cg**_ através do comando _cd cg_
4. Rode o mapa com o robô no pygame primeiramente pelo comando _colcon build_ seguido do comando _ros2 run cg maze_
5. Abra outro terminal e acesse a pasta deste repositório
6. Rode o método de navegação desejada, utilizando os comandos _colcon build_ seguido de _ros2 run jarc reativo_ ou _ros2 run jarc mapa_


