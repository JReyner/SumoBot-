"""Controlador ganador del sumobot: finalista 3, simulación del 17/09/2026.

Python 3 / lógica compatible con MicroPython, sin dependencias externas.
Entradas: cuatro sensores de línea (True = borde blanco), distancia en metros
(float('inf') si no se detecta rival) y dt en segundos.
Salidas: (motor_izquierdo, motor_derecho), valores normalizados entre -1 y 1.

No configura GPIO ni PWM y no carga firmware: conecta las lecturas de sensores
y las salidas de motor a un adaptador específico para tu placa.
Conserva el estado entre llamadas; llama reset() al iniciar cada combate.
"""
from math import sin

# Parámetros originales, con la precisión del resultado guardado.
# Ataque, búsqueda, alcance, retroceso, giro de escape, potencia de escape,
# oscilación, período, avance al buscar, memoria del objetivo, reintento cercano.
GENOME = (0.704878682057559, 0.64094701919198, 0.989551841530204, 0.0723303291797639, 0.465939131036401, 0.23828087952733, 0.0246961540970951, 3.06122203255557, 0.0, 0.98346078529954, 3.64379564806223)


class State:
    def __init__(self):
        self.reset()

    def reset(self):
        self.clock = 0.0
        self.escape = 0.0
        self.turn = 1.0
        self.seen = -100.0
        self.close_time = 0.0
        self.escape_sign = -1.0
        self.mode = "BUSCAR"


def bounded(value):
    return max(-1.0, min(1.0, value))


def drive(line, distance, state, dt):
    """Calcula las dos órdenes sin acceder al USB ni accionar motores.

    line: (delantero izquierdo, delantero derecho, trasero izquierdo,
           trasero derecho). dt: tiempo positivo desde la llamada anterior.
    """
    if len(line) != 4:
        raise ValueError("Se necesitan cuatro sensores de línea")
    if not 0.0 < dt < float("inf"):
        raise ValueError("dt debe ser positivo y finito, en segundos")
    if not distance >= 0.0:
        raise ValueError("Distancia no negativa o infinito si no hay objetivo")

    g = GENOME
    m = state
    m.clock += dt
    m.close_time = m.close_time + dt if distance < 0.10 else 0.0
    if m.close_time > g[10] and m.escape <= 0.0:
        m.close_time = 0.0
        m.escape = g[3] + g[4]
        m.escape_sign = -1.0
        m.turn *= -1.0

    if m.escape <= 0.0 and any(line):
        m.escape = g[3] + g[4]
        m.turn = -1.0 if line[0] or line[2] else 1.0
        m.escape_sign = -1.0 if line[0] or line[1] else 1.0

    if m.escape > 0.0:
        m.escape -= dt
        m.mode = "BORDE"
        if m.escape > g[4]:
            speed = g[5] * m.escape_sign
            return speed, speed
        return -m.turn * g[5], m.turn * g[5]

    if distance < g[2]:
        m.seen = m.clock
        m.mode = "EMPUJAR"
        weave = sin(m.clock * 6.283185307179586 / g[7]) * g[6]
        return bounded(g[0] - weave), bounded(g[0] + weave)

    if m.clock - m.seen < g[9]:
        m.mode = "SEGUIR"
        return g[0], g[0]

    m.mode = "BUSCAR"
    return bounded(g[8] - g[1] * m.turn), bounded(g[8] + g[1] * m.turn)


if __name__ == "__main__":
    # Ejemplo con lecturas ficticias: no mueve el robot.
    state = State()
    left, right = drive((False, False, False, False), 0.30, state, 0.02)
    print("Modo:", state.mode)
    print("Motor izquierdo:", left, "Motor derecho:", right)
