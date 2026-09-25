"""Controlador ganador del sumobot: finalista 3, simulación del 17/09/2026.

Lógica Python 3 y adaptador CircuitPython para IdeaBoard.
Entradas: cuatro sensores de línea (True = borde blanco), distancia en metros
(float('inf') si no se detecta rival) y dt en segundos.
Salidas: (motor_izquierdo, motor_derecho), valores normalizados entre -1 y 1.

Al ejecutarse en CircuitPython configura los GPIO documentados y espera BOOT.
Al importarse en Python de PC solo define funciones: no abre el puerto USB.
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



# Adaptador CircuitPython para IdeaBoard, según Conexiones SumoBot.pdf.
# Motores: M1 IO12/14 (izquierdo), M2 IO13/15 (derecho).
# IR: IO36, IO39, IO34, IO35. HC-SR04: TRIG IO25, ECHO IO26.
# Los signos dependen de la polaridad física de los cables del motor.
IR_THRESHOLDS = (10000, 10000, 10000, 10000)
MOTOR_SIGNS = (1, 1)
START_DELAY = 5.0
MATCH_SECONDS = 180.0
LOOP_SECONDS = 0.02


def run_robot():
    import board
    import analogio
    import digitalio
    import pwmio
    import pulseio
    import time

    outputs = []
    sensors = []
    button = None
    trigger = None
    echo = None

    def stop():
        for output in outputs:
            output.duty_cycle = 0

    def motors(left, right):
        # Dejar ambos terminales en cero antes de cambiar la dirección.
        stop()
        for i, value in enumerate((left, right)):
            value = bounded(value * MOTOR_SIGNS[i])
            outputs[2*i if value >= 0 else 2*i+1].duty_cycle = int(abs(value)*65535)

    def distance_m():
        # Captura limitada a 12 ms (~2 m), sin bloqueos largos sin eco.
        echo.pause()
        echo.clear()
        echo.resume()
        trigger.value = True
        time.sleep(0.00001)
        trigger.value = False
        deadline = time.monotonic() + 0.012
        while len(echo) == 0 and time.monotonic() < deadline:
            pass
        echo.pause()
        if len(echo) == 0:
            return float("inf")
        width = echo.popleft()
        if width < 100 or width > 12000:
            return float("inf")
        return width * 0.0001715

    try:
        # Inicialización con motores parados, usando la frecuencia de IdeaBoard.
        for pin in (board.IO12, board.IO14, board.IO13, board.IO15):
            outputs.append(pwmio.PWMOut(pin, duty_cycle=0, frequency=50))
        stop()
        for pin in (board.IO36, board.IO39, board.IO34, board.IO35):
            sensors.append(analogio.AnalogIn(pin))
        button = digitalio.DigitalInOut(board.IO0)
        button.switch_to_input(pull=digitalio.Pull.UP)
        trigger = digitalio.DigitalInOut(board.IO25)
        trigger.switch_to_output(value=False)
        echo = pulseio.PulseIn(board.IO26, maxlen=2, idle_state=False)
        echo.pause()
        state = State()
        phase = "ESPERA"
        deadline = 0.0
        match_deadline = 0.0
        last_step = time.monotonic()
        last_sonar = -1.0
        last_report = -1.0
        distance = float("inf")
        # Un botón ya pulsado al arrancar no inicia una partida.
        previous_button = button.value
        stable_button = previous_button
        changed_at = last_step
        print("SUMOBOT GANADOR LISTO. Motores parados. BOOT: iniciar tras 5 s; BOOT otra vez: parar.")
        while True:
            now = time.monotonic()
            raw_button = button.value
            if raw_button != previous_button:
                previous_button = raw_button
                changed_at = now
            pressed = False
            if raw_button != stable_button and now-changed_at >= 0.04:
                stable_button = raw_button
                pressed = not stable_button
            values = tuple(sensor.value for sensor in sensors)
            line = tuple(values[i] < IR_THRESHOLDS[i] for i in range(4))
            if pressed:
                stop()
                if phase == "ESPERA":
                    phase = "CUENTA"
                    deadline = now + START_DELAY
                    print("Inicio en 5 segundos. BOOT cancela.")
                else:
                    phase = "ESPERA"
                    print("PARADO")
            if phase == "CUENTA" and now >= deadline:
                if any(line):
                    phase = "ESPERA"
                    print("Inicio cancelado: hay sensores en blanco. Revisa suelo y umbrales.")
                else:
                    phase = "COMBATE"
                    state.reset()
                    last_step = now
                    match_deadline = now + MATCH_SECONDS
                    last_sonar = -1.0
                    print("COMBATE")
            if now-last_sonar >= 0.06:
                distance = distance_m()
                last_sonar = now
            if phase == "COMBATE":
                dt = now-last_step
                if now >= match_deadline or dt > 0.15:
                    stop()
                    phase = "ESPERA"
                    print("PARADO: tiempo máximo o ciclo lento")
                elif dt > 0:
                    # Releer bordes después de la medición ultrasónica.
                    line = tuple(sensors[i].value < IR_THRESHOLDS[i] for i in range(4))
                    left, right = drive(line, distance, state, dt)
                    motors(left, right)
            else:
                stop()
            last_step = now
            if phase != "COMBATE" and now-last_report >= 1.0:
                print("ESTADO", phase, "IR", values, "BLANCO", line, "DIST_M", distance)
                last_report = now
            time.sleep(max(0.001, LOOP_SECONDS-(time.monotonic()-now)))
    finally:
        stop()
        for resource in [echo, trigger, button] + sensors + outputs:
            if resource is not None:
                resource.deinit()


if __name__ == "__main__":
    run_robot()
