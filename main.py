#!/usr/bin/env pybricks-micropython
# -*- coding: utf-8 -*-
#kkkkk
"""
Labirinto EV3 (Pybricks / MicroPython)
O robô APRENDE (explora), MEMORIZA (salva a matriz em arquivo) e depois
percorre o menor caminho com DIJKSTRA usando MATRIZ DE ADJACÊNCIA.

Botões no bloco EV3:
  ESQUERDA -> modo APRENDER : explora o labirinto (DFS), monta e salva a matriz
  DIREITA  -> modo EXECUTAR : carrega a matriz salva, roda Dijkstra e percorre

Hipóteses (ajuste nas constantes):
  - Labirinto em grade LINHAS x COLUNAS, células quadradas de TAMANHO_CELULA mm
  - Robô começa no centro da célula INICIO, olhando para DIRECAO_INICIAL
  - Sensor ultrassônico aponta para a frente, no centro do robô
"""
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# ---------------------------------------------------------------- CONFIG ---
LINHAS = 4
COLUNAS = 4
TAMANHO_CELULA = 300          # mm (CALIBRE com seu labirinto)
LIMIAR_PAREDE = TAMANHO_CELULA  # distância < limiar => parede à frente
INICIO = (0, 0)
FIM = (LINHAS - 1, COLUNAS - 1)
DIRECAO_INICIAL = 0           # 0=Norte, 1=Leste, 2=Sul, 3=Oeste
ARQUIVO = "labirinto.txt"

INF = 9999                    # "infinito" (sem ligação / parede)
DIRECOES = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # N, L, S, O

# -------------------------------------------------------------- HARDWARE ---
ev3 = EV3Brick()
motor_esq = Motor(Port.B)
motor_dir = Motor(Port.C)
sensor = UltrasonicSensor(Port.S4)
robo = DriveBase(motor_esq, motor_dir, wheel_diameter=55.5, axle_track=104)
robo.settings(150, 150, 90, 90)

# Estado atual do robô (posição na grade e direção)
pos = [INICIO[0], INICIO[1]]
direcao = [DIRECAO_INICIAL]


# ------------------------------------------------ MATRIZ DE ADJACÊNCIA ------
def indice(l, c):
    return l * COLUNAS + c


def posicao(i):
    return i // COLUNAS, i % COLUNAS


def criar_matriz():
    n = LINHAS * COLUNAS
    matriz = [[INF] * n for _ in range(n)]
    for i in range(n):
        matriz[i][i] = 0
    return matriz


def conectar(matriz, a, b, peso=1):
    matriz[a][b] = peso
    matriz[b][a] = peso


# ------------------------------------------------------- MEMÓRIA (ARQUIVO) --
def salvar(matriz):
    with open(ARQUIVO, "w") as f:
        f.write("{} {}\n".format(LINHAS, COLUNAS))
        for linha in matriz:
            f.write(" ".join(str(v) for v in linha) + "\n")


def carregar():
    with open(ARQUIVO) as f:
        linhas = [x for x in f.read().split("\n") if x.strip()]
    dims = linhas[0].split()
    if int(dims[0]) != LINHAS or int(dims[1]) != COLUNAS:
        raise ValueError("Tamanho do labirinto salvo e diferente do configurado")
    return [[int(v) for v in x.split()] for x in linhas[1:]]


# ----------------------------------------------------------- MOVIMENTOS -----
def virar_para(d):
    giro = (d - direcao[0]) % 4
    if giro == 1:
        robo.turn(90)
    elif giro == 2:
        robo.turn(180)
    elif giro == 3:
        robo.turn(-90)
    direcao[0] = d


def avancar():
    robo.straight(TAMANHO_CELULA)
    dl, dc = DIRECOES[direcao[0]]
    pos[0] += dl
    pos[1] += dc


def livre_a_frente():
    """Mede 3 vezes e usa a menor leitura para reduzir ruído."""
    menor = 99999
    for _ in range(3):
        wait(60)
        menor = min(menor, sensor.distance())
    return menor > LIMIAR_PAREDE


# ------------------------------------------- APRENDER: exploração (DFS) -----
def explorar(l, c, matriz, visitado):
    visitado[l][c] = True
    livres = []

    # 1) olha as 4 direções e registra passagens (paredes ficam como INF)
    for d in range(4):
        dl, dc = DIRECOES[d]
        nl, nc = l + dl, c + dc
        if not (0 <= nl < LINHAS and 0 <= nc < COLUNAS):
            continue  # borda do labirinto
        virar_para(d)
        if livre_a_frente():
            conectar(matriz, indice(l, c), indice(nl, nc))
            livres.append(d)

    # 2) visita vizinhos livres ainda não explorados e volta (backtracking)
    for d in livres:
        dl, dc = DIRECOES[d]
        nl, nc = l + dl, c + dc
        if not visitado[nl][nc]:
            virar_para(d)
            avancar()
            explorar(nl, nc, matriz, visitado)
            virar_para((d + 2) % 4)
            avancar()  # volta para (l, c)


def modo_aprender():
    ev3.screen.clear()
    ev3.screen.print("Explorando...")
    matriz = criar_matriz()
    visitado = [[False] * COLUNAS for _ in range(LINHAS)]
    explorar(INICIO[0], INICIO[1], matriz, visitado)
    virar_para(DIRECAO_INICIAL)
    salvar(matriz)
    ev3.speaker.beep(1000, 300)
    ev3.screen.print("Mapa salvo!")


# --------------------------------------------------------- DIJKSTRA O(V^2) --
def dijkstra(matriz, origem, destino):
    n = len(matriz)
    dist = [INF] * n
    anterior = [None] * n
    visitado = [False] * n
    dist[origem] = 0

    for _ in range(n):
        u, menor = -1, INF
        for i in range(n):
            if not visitado[i] and dist[i] < menor:
                menor, u = dist[i], i
        if u == -1 or u == destino:
            break
        visitado[u] = True
        for v in range(n):
            peso = matriz[u][v]
            if v != u and peso < INF and not visitado[v]:
                if dist[u] + peso < dist[v]:
                    dist[v] = dist[u] + peso
                    anterior[v] = u

    if dist[destino] >= INF:
        return None
    caminho = []
    atual = destino
    while atual is not None:
        caminho.append(atual)
        atual = anterior[atual]
    caminho.reverse()
    return caminho


# ---------------------------------------------------- EXECUTAR o caminho ----
def modo_executar():
    try:
        matriz = carregar()
    except (OSError, ValueError):
        ev3.speaker.beep(200, 500)
        ev3.screen.print("Sem mapa salvo!")
        return

    caminho = dijkstra(matriz, indice(*INICIO), indice(*FIM))
    if caminho is None:
        ev3.speaker.beep(200, 500)
        ev3.screen.print("Sem caminho!")
        return

    ev3.screen.print("Indo ao fim...")
    for a, b in zip(caminho, caminho[1:]):
        la, ca = posicao(a)
        lb, cb = posicao(b)
        d = DIRECOES.index((lb - la, cb - ca))
        virar_para(d)
        avancar()
    ev3.speaker.beep(1500, 300)
    ev3.screen.print("Chegou!")


# ------------------------------------------------------------------ MAIN ----
ev3.speaker.beep()
ev3.screen.print("ESQ: aprender")
ev3.screen.print("DIR: executar")

while True:
    botoes = ev3.buttons.pressed()
    if Button.LEFT in botoes:
        wait(400)
        modo_aprender()
        break
    if Button.RIGHT in botoes:
        wait(400)
        modo_executar()
        break
    wait(50)