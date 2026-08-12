# safaryv2

## Notas operativas y de seguridad

- En `IDLE`, al mantener logo:
  - modo `WEARABLE` entra en `VIGILANCIA`
  - modo `VELADOR` entra en `SLEEP`
- El disparo por proximidad ultrasónica se usa solo en `VELADOR` para reducir falsos positivos en uso corporal.
- El análisis post-evento usa `TIMEOUT_ANALISIS_MS` como salida de fallback real cuando no hay evidencia clara.
- La medición ultrasónica está cacheada y usa menos intentos en vigilancia para evitar bloquear el muestreo del acelerómetro.
- **Hardware crítico:** si usas HC-SR04 clásico, el pin `ECHO` (5V) **no** debe conectarse directo a la micro:bit (3.3V). Usa divisor de tensión o level shifter.