# ECOM062-Robotica-Projeto-Final

Esse repositório contêm o projeto final da disciplina de Robótica ministrada pelo Prof. Glauber.

Desenvolvemos em conjunto, uma aplicação muito comum de pick and place utilizando como base o braço robótio Denso VP-6242 em conjunto com uma modelo de garra da Robotiq 2F-85. Utilizamos de técnicas vistas em sala de aula, como calculo de cinemática direta e inversa, jacobina, workspace e afins, tomando como base o livro do Bruno Sicilliano.

Para rodar código vá até a pasta `src/main.py` e rode o arquivo. É importante que os arquivos `remoteApi.so`, `sim.py`, e `simConst.py` estejam no mesmo diretório da main, já que a biblioteca `sim` dentro da main, depende desses arquivos para funcionar corretamente.