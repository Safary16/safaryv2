from microbit import *
import utime
import math

# ==============================================================================
# 🦖 PROYECTO SAFARY V2: IMPLEMENTACIÓN DE SUPERVIVENCIA (ULTRASONIDO + BUZZER)
# ==============================================================================

# --- CONFIGURACIÓN DE INGENIERÍA ---
UMBRAL_IMPACTO = 2.5
T_QUIETUD_DESMAYO = 0.0005
T_SILENCIO_POST_EVENTO = 0.05
TIEMPO_PRE_ALERTA_MS = 15000
VENTANA_TIEMPO_MS = 5000
VENTANA_ANALISIS_MIN_MS = 2000
TIMEOUT_ANALISIS_MS = 5000
INTERVALO_MUESTREO_SLEEP = 200
MAX_BUFFER_MUESTRAS = 500
UMBRAL_DESPERTAR = 1.3
INTERVALO_MUESTREO_VIGILANCIA_MS = 10

# --- NUEVO HARDWARE ---
PIN_BUZZER = pin15
PIN_ULTRA_TRIG = pin13
PIN_ULTRA_ECHO = pin14
UMBRAL_DISTANCIA_CERCANA_CM = 70
UMBRAL_DESCENSO_RAPIDO_CM = 25
VENTANA_DESCENSO_RAPIDO_MS = 1200
T_ESTABILIDAD_DISTANCIA_CM = 6

# --- ESTADOS FSM ---
ESTADO_IDLE = "IDLE"
ESTADO_SLEEP_WATCH = "SLEEP"
ESTADO_VIGILANCIA = "VIGILANCIA"
ESTADO_ANALISIS_POST_EVENTO = "ANALISIS"
ESTADO_PRE_ALERTA = "PRE_ALERTA"
ESTADO_ALARMA = "ALARMA"
ESTADO_RESCATE = "RESCATE"

# --- VARIABLES GLOBALES ---
estado_actual = ESTADO_IDLE
buffer_aceleracion = []
buffer_distancia = []
tiempo_inicio_quietud = 0
tiempo_quietud_acumulado = 0
tiempo_entrada_estado = 0
es_primera_iteracion = True
tiempo_touch_logo = 0
tiempo_touch_logo_sos = None
g_smooth = 1.0
peak_g_impact = 0.0
tiempo_panic_trigger = 0
contador_varianza = 0
evento_disparador = None  # "impacto" o "proximidad"

# Secuenciador Morse SOS
morse_index = 0
morse_tiempo_cambio = 0
morse_pattern = [
    (1, 150), (0, 150), (1, 150), (0, 150), (1, 150), (0, 300),
    (1, 450), (0, 150), (1, 450), (0, 150), (1, 450), (0, 300),
    (1, 150), (0, 150), (1, 150), (0, 150), (1, 150), (0, 1000)
]


def get_g_force_raw():
    x, y, z = accelerometer.get_x(), accelerometer.get_y(), accelerometer.get_z()
    return math.sqrt(x**2 + y**2 + z**2) / 1000.0


def get_g_force():
    global g_smooth
    mag_raw = get_g_force_raw()
    g_smooth = (0.9 * g_smooth) + (0.1 * mag_raw)
    return mag_raw, g_smooth


def medir_distancia_cm():
    # Pulso de disparo de 10us para HC-SR04/compatible
    PIN_ULTRA_TRIG.write_digital(0)
    utime.sleep_us(2)
    PIN_ULTRA_TRIG.write_digital(1)
    utime.sleep_us(10)
    PIN_ULTRA_TRIG.write_digital(0)

    timeout = 30000
    t0 = utime.ticks_us()
    while PIN_ULTRA_ECHO.read_digital() == 0:
        if utime.ticks_diff(utime.ticks_us(), t0) > timeout:
            return None

    inicio = utime.ticks_us()
    while PIN_ULTRA_ECHO.read_digital() == 1:
        if utime.ticks_diff(utime.ticks_us(), inicio) > timeout:
            return None

    fin = utime.ticks_us()
    duracion = utime.ticks_diff(fin, inicio)
    return duracion / 58.0


def calcular_varianza(buffer):
    n = len(buffer)
    if n < 2:
        return 0
    suma_total = 0.0
    for t, v in buffer:
        suma_total += v
    media = suma_total / n
    suma_cuadrados = 0.0
    for t, v in buffer:
        suma_cuadrados += (v - media) ** 2
    return suma_cuadrados / n


def gestionar_buffer(g_force):
    ahora = utime.ticks_ms()
    buffer_aceleracion.append((ahora, g_force))
    while buffer_aceleracion and utime.ticks_diff(ahora, buffer_aceleracion[0][0]) > VENTANA_TIEMPO_MS:
        buffer_aceleracion.pop(0)
    while len(buffer_aceleracion) > MAX_BUFFER_MUESTRAS:
        buffer_aceleracion.pop(0)


def gestionar_buffer_distancia(distancia_cm):
    if distancia_cm is None:
        return
    ahora = utime.ticks_ms()
    buffer_distancia.append((ahora, distancia_cm))
    while buffer_distancia and utime.ticks_diff(ahora, buffer_distancia[0][0]) > VENTANA_TIEMPO_MS:
        buffer_distancia.pop(0)
    while len(buffer_distancia) > MAX_BUFFER_MUESTRAS:
        buffer_distancia.pop(0)


def descenso_distancia_rapido():
    if len(buffer_distancia) < 2:
        return False
    ahora = utime.ticks_ms()
    t_actual, d_actual = buffer_distancia[-1]
    if d_actual > UMBRAL_DISTANCIA_CERCANA_CM:
        return False

    for i in range(len(buffer_distancia) - 2, -1, -1):
        t_prev, d_prev = buffer_distancia[i]
        if utime.ticks_diff(ahora, t_prev) > VENTANA_DESCENSO_RAPIDO_MS:
            break
        if (d_prev - d_actual) >= UMBRAL_DESCENSO_RAPIDO_CM:
            return True
    return False


def distancia_estable_post_evento():
    if len(buffer_distancia) < 5:
        return False
    valores = []
    for t, d in buffer_distancia:
        valores.append(d)
    return (max(valores) - min(valores)) <= T_ESTABILIDAD_DISTANCIA_CM


def buzzer_on():
    PIN_BUZZER.write_digital(1)


def buzzer_off():
    PIN_BUZZER.write_digital(0)


def sonido_suave():
    buzzer_on()
    utime.sleep_ms(120)
    buzzer_off()
    utime.sleep_ms(80)
    buzzer_on()
    utime.sleep_ms(120)
    buzzer_off()


def tick():
    global estado_actual, es_primera_iteracion
    global tiempo_inicio_quietud, tiempo_quietud_acumulado
    global tiempo_entrada_estado, buffer_aceleracion, buffer_distancia, tiempo_touch_logo
    global tiempo_touch_logo_sos, morse_index, morse_tiempo_cambio
    global peak_g_impact, tiempo_panic_trigger, contador_varianza, evento_disparador

    ahora = utime.ticks_ms()

    if button_a.is_pressed() and button_b.is_pressed():
        if estado_actual not in (ESTADO_ALARMA, ESTADO_RESCATE):
            if tiempo_panic_trigger == 0:
                tiempo_panic_trigger = ahora
            elif utime.ticks_diff(ahora, tiempo_panic_trigger) > 1500:
                estado_actual = ESTADO_ALARMA
                es_primera_iteracion = True
                tiempo_panic_trigger = 0
                return
    else:
        tiempo_panic_trigger = 0

    if estado_actual == ESTADO_IDLE:
        if es_primera_iteracion:
            display.show(Image.ASLEEP)
            es_primera_iteracion = False
            tiempo_touch_logo = 0

        if pin_logo.is_touched():
            if tiempo_touch_logo == 0:
                tiempo_touch_logo = ahora
            elif utime.ticks_diff(ahora, tiempo_touch_logo) > 1500:
                estado_actual = ESTADO_SLEEP_WATCH
                es_primera_iteracion = True
                utime.sleep_ms(500)
        else:
            tiempo_touch_logo = 0
            utime.sleep_ms(50)

    elif estado_actual == ESTADO_SLEEP_WATCH:
        if es_primera_iteracion:
            display.clear()
            es_primera_iteracion = False
            tiempo_touch_logo = 0
            buffer_distancia = []

        if pin_logo.is_touched():
            if tiempo_touch_logo == 0:
                tiempo_touch_logo = ahora
            elif utime.ticks_diff(ahora, tiempo_touch_logo) > 1500:
                estado_actual = ESTADO_IDLE
                es_primera_iteracion = True
                utime.sleep_ms(500)
                return
        else:
            tiempo_touch_logo = 0

        utime.sleep_ms(INTERVALO_MUESTREO_SLEEP)

        g_raw, g_smooth_unused = get_g_force()
        distancia = medir_distancia_cm()
        gestionar_buffer_distancia(distancia)

        if g_raw > UMBRAL_DESPERTAR:
            evento_disparador = "impacto"
            peak_g_impact = g_raw
            estado_actual = ESTADO_ANALISIS_POST_EVENTO
            es_primera_iteracion = True
        elif descenso_distancia_rapido():
            evento_disparador = "proximidad"
            peak_g_impact = max(peak_g_impact, g_raw)
            estado_actual = ESTADO_ANALISIS_POST_EVENTO
            es_primera_iteracion = True

    elif estado_actual == ESTADO_VIGILANCIA:
        if es_primera_iteracion:
            tiempo_inicio_quietud = 0
            tiempo_quietud_acumulado = 0
            contador_varianza = 0
            display.show(Image.HAPPY)
            es_primera_iteracion = False

        if pin_logo.is_touched():
            estado_actual = ESTADO_SLEEP_WATCH
            es_primera_iteracion = True
            utime.sleep_ms(500)
            return

        utime.sleep_ms(INTERVALO_MUESTREO_VIGILANCIA_MS)
        g_raw, g_smooth = get_g_force()
        distancia = medir_distancia_cm()
        gestionar_buffer(g_smooth)
        gestionar_buffer_distancia(distancia)

        if g_raw > UMBRAL_IMPACTO:
            evento_disparador = "impacto"
            peak_g_impact = g_raw
            estado_actual = ESTADO_ANALISIS_POST_EVENTO
            es_primera_iteracion = True
            return

        if descenso_distancia_rapido():
            evento_disparador = "proximidad"
            estado_actual = ESTADO_ANALISIS_POST_EVENTO
            es_primera_iteracion = True
            return

        contador_varianza += 1
        if contador_varianza >= 50:
            contador_varianza = 0
            varianza = calcular_varianza(buffer_aceleracion)
            if len(buffer_aceleracion) >= 10:
                if varianza < T_QUIETUD_DESMAYO:
                    if tiempo_inicio_quietud == 0:
                        tiempo_inicio_quietud = ahora
                    tiempo_quietud_acumulado = utime.ticks_diff(ahora, tiempo_inicio_quietud)
                    if tiempo_quietud_acumulado > 30000:
                        estado_actual = ESTADO_PRE_ALERTA
                        es_primera_iteracion = True
                        return
                else:
                    tiempo_inicio_quietud = 0
                    tiempo_quietud_acumulado = 0

    elif estado_actual == ESTADO_ANALISIS_POST_EVENTO:
        if es_primera_iteracion:
            buffer_aceleracion = []
            buffer_distancia = []
            tiempo_entrada_estado = ahora
            display.show(Image.TARGET)
            es_primera_iteracion = False

        utime.sleep_ms(INTERVALO_MUESTREO_VIGILANCIA_MS)
        g_raw, g_smooth = get_g_force()
        distancia = medir_distancia_cm()

        if utime.ticks_diff(ahora, tiempo_entrada_estado) < 500:
            return

        gestionar_buffer(g_raw)
        gestionar_buffer_distancia(distancia)

        if utime.ticks_diff(ahora, tiempo_entrada_estado) >= (VENTANA_ANALISIS_MIN_MS + 500):
            v_actual = calcular_varianza(buffer_aceleracion)
            distancia_estable = distancia_estable_post_evento()

            if evento_disparador == "proximidad":
                evento_coherente = distancia_estable
            else:
                evento_coherente = True

            if v_actual < T_SILENCIO_POST_EVENTO and evento_coherente:
                estado_actual = ESTADO_PRE_ALERTA
                es_primera_iteracion = True
            else:
                estado_actual = ESTADO_VIGILANCIA
                es_primera_iteracion = True
            return

        if utime.ticks_diff(ahora, tiempo_entrada_estado) > TIMEOUT_ANALISIS_MS:
            estado_actual = ESTADO_VIGILANCIA
            es_primera_iteracion = True
            return

    elif estado_actual == ESTADO_PRE_ALERTA:
        if es_primera_iteracion:
            tiempo_entrada_estado = ahora
            display.show("?")
            sonido_suave()
            es_primera_iteracion = False

        if (button_a.is_pressed() or button_b.is_pressed()) and not (button_a.is_pressed() and button_b.is_pressed()):
            estado_actual = ESTADO_VIGILANCIA
            es_primera_iteracion = True
        elif utime.ticks_diff(ahora, tiempo_entrada_estado) > TIEMPO_PRE_ALERTA_MS:
            estado_actual = ESTADO_ALARMA
            es_primera_iteracion = True

    elif estado_actual == ESTADO_ALARMA:
        if es_primera_iteracion:
            morse_index = 0
            state, duration = morse_pattern[0]
            morse_tiempo_cambio = utime.ticks_add(ahora, duration)
            if state == 1:
                display.show(Image.SQUARE)
                buzzer_on()
            else:
                display.clear()
                buzzer_off()
            es_primera_iteracion = False

        if utime.ticks_diff(ahora, morse_tiempo_cambio) >= 0:
            morse_index = (morse_index + 1) % len(morse_pattern)
            state, duration = morse_pattern[morse_index]
            morse_tiempo_cambio = utime.ticks_add(ahora, duration)
            if state == 1:
                display.show(Image.SQUARE)
                buzzer_on()
            else:
                display.clear()
                buzzer_off()

        if button_a.is_pressed() and button_b.is_pressed() and pin_logo.is_touched():
            estado_actual = ESTADO_RESCATE
            es_primera_iteracion = True
            buzzer_off()

    elif estado_actual == ESTADO_RESCATE:
        if es_primera_iteracion:
            display.show(Image.YES)
            sonido_suave()
            utime.sleep_ms(2000)
            estado_actual = ESTADO_IDLE
            es_primera_iteracion = True


def main():
    accelerometer.set_range(8)
    PIN_BUZZER.write_digital(0)
    PIN_ULTRA_TRIG.write_digital(0)
    while True:
        tick()


if __name__ == '__main__':
    main()
