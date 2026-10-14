# Firmware — Moto Crash Guardian

Firmware del módulo que va en la moto. Lee un acelerómetro/giroscopio y un GPS, detecta posibles caídas y se lo informa por **Bluetooth Low Energy** a la app Android, que valida el evento, da una cuenta regresiva al motociclista y avisa a sus contactos.

> **Prototipo académico.** No es un sistema eCall certificado, no reemplaza a los servicios de emergencia y no garantiza detectar todos los accidentes. En Colombia, ante una emergencia, llama a la línea **123**.

El módulo **solo detecta y reporta**. La decisión de pedir ayuda (SMS, llamada) la toma la app en el teléfono; el módulo no necesita internet.

## Contenido

1. [Hardware](#1-hardware)
2. [Cableado](#2-cableado)
3. [Cómo funciona](#3-cómo-funciona)
4. [Protocolo BLE](#4-protocolo-ble)
5. [Preparar el entorno](#5-preparar-el-entorno)
6. [Compilar y cargar el firmware al ESP32](#6-compilar-y-cargar-el-firmware-al-esp32)
7. [Comprobar que funciona](#7-comprobar-que-funciona)
8. [Usarlo con la app](#8-usarlo-con-la-app)
9. [Pruebas unitarias](#9-pruebas-unitarias)
10. [Solución de problemas](#10-solución-de-problemas)
11. [Estado y límites conocidos](#11-estado-y-límites-conocidos)
12. [Estructura del repositorio](#12-estructura-del-repositorio)

---

## 1. Hardware

| Componente | Modelo | Notas |
|---|---|---|
| Microcontrolador | ESP32 DevKit (`esp32dev`) | BLE integrado; el WiFi se apaga a propósito |
| IMU | MPU-6050 (I²C, dirección `0x68`) | ±16 g y ±2000 °/s, 200 muestras/s |
| GPS | NEO-6M | NMEA a 9600 baudios, 1 actualización por segundo |
| Batería | LiPo 1S (3,3–4,2 V) | Se mide con un divisor resistivo 100 kΩ / 100 kΩ |

Necesitas además un **cable USB de datos** (algunos cables solo cargan y el ESP32 no aparecerá como puerto serie).

## 2. Cableado

Los pines salen de [`include/config.h`](include/config.h).

| Señal | Pin del ESP32 | Conecta a |
|---|---|---|
| I²C SDA | GPIO 21 | MPU-6050 SDA |
| I²C SCL | GPIO 22 | MPU-6050 SCL |
| MPU INT | GPIO 19 | MPU-6050 INT (definido, **no usado**) |
| GPS RX | GPIO 16 | NEO-6M **TX** |
| GPS TX | GPIO 17 | NEO-6M **RX** (opcional) |
| Batería (ADC) | GPIO 34 | Punto medio del divisor 100 kΩ / 100 kΩ |
| Estado de carga | GPIO 35 | Señal CHRG del cargador (definido, **no implementado**) |
| LED | GPIO 2 | LED integrado (definido, **no usado**) |

- Alimenta el MPU-6050 y el NEO-6M a **3,3 V** y une las tierras (GND) de todos los módulos.
- Los RX/TX van **cruzados**: el TX del GPS al RX del ESP32.
- ⚠️ Los pines de `config.h` están marcados *por confirmar con el cableado real*. Si tu montaje es distinto, **cambia los números en `config.h`** y vuelve a compilar.
- El divisor de batería no debe superar 3,3 V en GPIO 34: con una LiPo de 4,2 V el divisor 1:2 da 2,1 V.

## 3. Cómo funciona

### Tareas (FreeRTOS)

| Tarea | Núcleo | Prioridad | Frecuencia | Qué hace |
|---|---|---|---|---|
| `imuTask` | 1 | 5 | 200 Hz | Lee el IMU, filtra la orientación, ejecuta el detector, llena el anillo de la caja negra y atiende los comandos |
| `bleServerTask` | 0 | 3 | 10 ms | Servidor BLE: telemetría (5 Hz), GPS (1 Hz), eventos, batería y envío de trazas |
| `gpsTask` | 0 | 2 | continua | Lee NMEA y publica una foto cada segundo |
| `powerTask` | 0 | 1 | 1 s | Mide la batería (curva LiPo 1S) |

### Estados del detector

```
BOOT → DISARMED ──ARM──► ARMED ──disparo──► POST_TRIGGER (3 s) ──► COOLDOWN (10 s) ──► ARMED
                            ▲                                                          │
                            └──────────────────────────────────────────────────────────┘
                 (FAULT si el IMU deja de responder; se recupera solo y vuelve a DISARMED)
```

- **El módulo arranca `DISARMED` y solo detecta estando `ARMED`.** Se arma con el comando `ARM`, que **envía la app** al iniciar un viaje. Si lo pruebas sin la app, no habrá eventos de caída.
- **Candidatos a accidente:**
  - *Impacto*: aceleración ≥ umbral de impacto (por defecto 3,5 g) o velocidad angular ≥ umbral de giro (por defecto 300 °/s).
  - *Inclinación*: ángulo (cabeceo o alabeo) ≥ umbral (por defecto 65°) sostenido el tiempo configurado (por defecto 2 s). No se dispara si hubo un impacto en los 5 s anteriores, porque ese impacto ya generó su propio evento.
- El módulo emite el candidato; **la validación final (cuenta regresiva, cancelación) la hace la app.**

### Caja negra

El anillo guarda continuamente los últimos 3 s (600 muestras a 200 Hz). Al disparar, se conserva ese pasado y se capturan 3 s más: **1200 muestras (6 s) por evento**, cada una de 12 bytes (`ax, ay, az, gx, gy, gz` como enteros de 16 bits). Se guardan en RAM las **2 últimas trazas**; se pierden al reiniciar el módulo.

### Datos que se guardan en memoria no volátil (NVS)

Contador de arranques, umbrales de detección, revisión de la configuración, offsets de nivelación (TARE) y sesgo del giroscopio.

## 4. Protocolo BLE

El módulo se anuncia como **`MCG-XXXX`** (los dos últimos bytes de su dirección). Todas las tramas son *little-endian*. La definición exacta de cada trama está en [`lib/protocol/protocol.h`](lib/protocol/protocol.h) y sus vectores de prueba en [`test/test_protocol`](test/test_protocol/test_protocol_vectors.cpp).

Servicio principal: `a3c1f000-5b2e-4d7a-9f41-6c2e8b7d0e10`

| Característica | UUID (últimos 4) | Acceso | Contenido |
|---|---|---|---|
| `DEVICE_INFO` | `…f001` | lectura | Versión de protocolo y firmware, capacidades, `bootCount`, MAC (16 B) |
| `TELEMETRY` | `…f002` | notificación 5 Hz | Estado, flags, ángulos, aceleración, picos, batería (16 B) |
| `EVENT` | `…f003` | indicación | Evento de caída/prueba/fallo (20 B), con confirmación |
| `CONFIG` | `…f004` | lectura/escritura **cifrada** | Umbrales de detección y offsets (16 B) |
| `CONTROL_POINT` | `…f005` | escritura **cifrada** + indicación | Comandos y sus respuestas |
| `GPS` | `…f006` | notificación 1 Hz | Posición, velocidad, calidad (16 B) |
| `TRACE` | `…f007` | notificación | Chunks de la caja negra |
| Batería (estándar `0x180F`) | `0x2A19` | lectura/notificación | Porcentaje |

**Seguridad:** emparejamiento *Just Works* con *bonding* (LE Secure Connections). `CONFIG` y `CONTROL_POINT` exigen conexión cifrada.

### Comandos (`CONTROL_POINT`)

Formato de la solicitud: `[opcode, requestId, parámetros…]`. Respuesta: `[opcode, requestId, resultado, payload…]`.

| Opcode | Comando | Efecto |
|---|---|---|
| `0x01` | `TARE` | Fija el nivel neutro. Exige que la moto esté quieta; si no, responde `BUSY` |
| `0x02` | `ARM` | Activa la detección |
| `0x03` | `DISARM` | Desactiva la detección |
| `0x04` | `TRIGGER_TEST_EVENT` | Emite un evento `TEST` (para simulacros). **No tiene traza** |
| `0x05` | `REQUEST_TRACE` | Pide la caja negra de un evento: responde con los metadatos y luego envía los chunks por `TRACE` |
| `0x06` | `ACK_EVENT` | Confirma la recepción de un evento (`bootCount`, `eventId`) |
| `0x07` | `REBOOT` | Reinicia el módulo |
| `0x08` | `FACTORY_RESET` | Borra la configuración guardada; exige el parámetro `0xDEAD` |

Resultados: `0` OK · `1` no soportado · `2` parámetro inválido · `3` ocupado · `4` falló · `5` no encontrado.

### Eventos y claves

Cada evento lleva `bootCount` (que **se incrementa en cada arranque**) y `eventId` (que **se reinicia en cada arranque**). La clave única de un evento es `bootCount:eventId`. La traza de un evento solo se puede pedir **mientras el módulo no se haya reiniciado**, y no antes de ~3 s después del disparo (hasta que termina de capturar, responde `NOT_FOUND`).

Si no hay un cliente suscrito, hasta 8 eventos quedan en cola y se reenvían al reconectar marcados con el flag *reenviado*.

## 5. Preparar el entorno

Se compila con **PlatformIO**. Elige una de las dos formas.

### Opción A — Visual Studio Code (recomendada)

1. Instala [Visual Studio Code](https://code.visualstudio.com/).
2. En *Extensiones* busca e instala **PlatformIO IDE**. Normalmente instala por su cuenta Python y las herramientas de compilación, así que no hace falta tenerlos antes. La primera vez tarda unos minutos.
3. Reinicia VS Code cuando lo pida.
4. Abre la carpeta del firmware (`File → Open Folder…`, la carpeta que contiene `platformio.ini`).

### Opción B — Línea de comandos

1. Instala [Python 3.8 o superior](https://www.python.org/downloads/) (marca *Add Python to PATH* en Windows).
2. Instala PlatformIO Core:

   ```powershell
   pip install -U platformio
   pio --version
   ```

### Driver USB

La mayoría de las placas ESP32 DevKit usan un chip **CP210x** (Silicon Labs) o **CH340/CH341**. Si al conectar la placa no aparece ningún puerto COM, instala el driver correspondiente:

- CP210x: <https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers>
- CH340: busca "CH341SER" en el sitio del fabricante (WCH).

Comprueba en el *Administrador de dispositivos* → *Puertos (COM y LPT)* que aparece, por ejemplo, `Silicon Labs CP210x … (COM5)`.

## 6. Compilar y cargar el firmware al ESP32

1. **Conecta** la placa por USB al computador (con un cable de datos).
2. **Revisa los pines** de [`include/config.h`](include/config.h) y ajústalos si tu cableado es distinto (ver [sección 2](#2-cableado)).
3. **Compila** (la primera vez descarga el compilador y las librerías; necesita internet):

   - VS Code: botón **✓ Build** de la barra inferior de PlatformIO, o `Ctrl+Alt+B`.
   - Terminal:

     ```powershell
     pio run -e esp32dev
     ```

   Debe terminar con `SUCCESS`. Las librerías `NimBLE-Arduino` y `TinyGPSPlus` se instalan solas (están en `platformio.ini`).

4. **Carga** el firmware:

   - VS Code: botón **→ Upload**, o `Ctrl+Alt+U`.
   - Terminal:

     ```powershell
     pio run -e esp32dev -t upload
     ```

   PlatformIO detecta el puerto solo. Si hay varios dispositivos conectados, indícalo: `pio run -e esp32dev -t upload --upload-port COM5`.

5. **Si se queda en `Connecting........_____`**, la placa no entró en modo de programación. Mantén pulsado el botón **BOOT** de la placa mientras aparece `Connecting…` y suéltalo cuando empiece a escribir (`Writing at 0x…`). Algunas placas lo hacen solas; otras lo necesitan.

6. **Abre el monitor serie** para ver que arrancó:

   - VS Code: botón **🔌 Serial Monitor**.
   - Terminal: `pio device monitor -e esp32dev` (115200 baudios; `Ctrl+C` para salir).

   Si no ves nada, pulsa el botón **EN/RESET** de la placa.

> **Carga de un firmware nuevo:** repite los pasos 3–4. La configuración guardada (umbrales, TARE, contador de arranques) **se conserva** entre cargas. Para borrarla por completo: `pio run -e esp32dev -t erase` y vuelve a cargar.

## 7. Comprobar que funciona

Con el monitor serie abierto debes ver cada medio segundo un bloque como este (la telemetría de depuración):

```
--------------------------------------
GPS:
  Ubicacion: SIN FIX
IMU:
  Pitch: 0.4 deg
  Roll:  1.2 deg
  |a|: 1.01 g
  |w|: 12 deg/s
  Acel X: ...
  Estado: 1
--------------------------------------
```

| Qué mirar | Esperado |
|---|---|
| `|a|` con la placa quieta | ≈ **1,00 g** (la gravedad) |
| `Pitch` / `Roll` | Cambian al inclinar la placa |
| `Estado` | `1` = desarmado (normal tras encender). `2` = armado (cuando la app lo arma) |
| `GPS` | `SIN FIX` en interiores o los primeros minutos; con cielo abierto aparecen latitud, longitud y satélites. Un *cold start* del NEO-6M puede tardar varios minutos |
| Sin lecturas / `|a|` = 0 | El IMU no responde: revisa SDA/SCL, alimentación y dirección `0x68`. El módulo entra en estado de **fallo** (`Estado: 6`) y reintenta cada segundo |

Para desactivar esta salida (ahorra energía), pon `#define SERIAL_TELEMETRY 0` en `config.h`.

## 8. Usarlo con la app

1. Enciende el módulo. Se anuncia como `MCG-XXXX` (anuncio rápido durante 30 s y luego más lento).
2. En la app **Moto Crash Guardian**, completa el asistente de vinculación y elige el módulo `MCG-XXXX`. El emparejamiento es automático (*Just Works*).
3. **Con la moto quieta y nivelada**, haz el **TARE** desde la app (pestaña *Calibración*). Si la moto se mueve, el módulo responde `BUSY`.
4. **Inicia el viaje** en la app: ella arma el módulo (`ARM`) y, a partir de ahí, un impacto o una inclinación sostenida generan un evento.
5. Para ensayar sin provocar un choque, usa el **simulacro** de la app (usa `TRIGGER_TEST_EVENT`).

> Si en Android cambiaste de módulo o reflasheaste con `erase`, **olvida el dispositivo** en *Ajustes → Bluetooth* antes de volver a emparejar; si no, el teléfono conserva una clave antigua y la conexión cifrada falla.

## 9. Pruebas unitarias

La lógica pura (detector de candidatos y empaquetado del protocolo) se prueba en tu computador, sin la placa:

```powershell
pio test -e native
```

Requiere un compilador de C++ en el `PATH` (en Windows, MinGW-w64/`g++`; en Linux/macOS, `g++` o `clang`).

| Suite | Qué cubre |
|---|---|
| `test_detection` | Reposo, impacto, giro, inclinación, estados, *cooldown*, TARE, fallos |
| `test_protocol` | Tamaño de cada trama y vectores de bytes compartidos con la app |

Los vectores de `test_protocol` son los mismos que usa la app Android: el firmware los empaqueta y la app los interpreta byte a byte.

## 10. Solución de problemas

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| No aparece ningún puerto COM | Driver faltante o cable solo de carga | Instala el driver CP210x/CH340; prueba otro cable USB de datos y otro puerto |
| `Failed to connect to ESP32: Wrong boot mode` / `Connecting…` infinito | La placa no entró en modo de programación | Mantén **BOOT** mientras conecta (ver paso 5 de la [sección 6](#6-compilar-y-cargar-el-firmware-al-esp32)) |
| `Could not open COM5, the port is busy` | Otro programa usa el puerto (monitor serie, Arduino IDE) | Cierra el monitor serie antes de cargar |
| La carga falla a mitad (`A fatal error occurred: Packet content transfer stopped`) | Velocidad o cable inestable | Añade `upload_speed = 115200` en `[env:esp32dev]` de `platformio.ini` y usa un cable más corto |
| La compilación falla con `'class TwoWire' has no member named 'resetBus'` (u otro error de la API de Arduino) | El comentario de `platformio.ini` habla de Arduino-ESP32 3.x, pero la plataforma oficial `espressif32@6.x` trae el núcleo **2.0.x** | Usa el fork *pioarduino* con el núcleo 3.x: en `[env:esp32dev]` cambia `platform` por `https://github.com/pioarduino/platform-espressif32/releases/download/stable/platform-espressif32.zip`, o reemplaza esa llamada por el reinicio del bus de tu versión |
| Error por falta de la librería | Sin internet en el primer build | Conéctate a internet y repite `pio run`; las librerías se guardan en `.pio/libdeps` |
| El módulo no aparece al escanear | Ya está conectado a otro teléfono, o se apagó | Desconéctalo del otro dispositivo; reinicia el módulo |
| Se conecta pero los comandos fallan (`CONFIG`, `ARM`, `TARE`) | Claves de emparejamiento desfasadas | Olvida el dispositivo en Android y vuelve a emparejar |
| No hay eventos de caída | El módulo no está armado | La app lo arma al **iniciar el viaje**; comprueba `Estado: 2` en el monitor serie |
| `TARE` responde `BUSY` | La moto no está quieta (1 g ± 50 mg y ≤ 5 °/s durante 1 s) | Déjala inmóvil y repite |
| El GPS siempre dice `SIN FIX` | Sin vista al cielo, o TX/RX sin cruzar | Pruébalo en exterior varios minutos; revisa que el TX del GPS vaya al GPIO 16 |
| El porcentaje de batería es incorrecto | Divisor distinto o sin poblar | Ajusta `BATTERY_DIVIDER_RATIO` o pon `BATTERY_SENSING_ENABLED 0` (reporta "desconocida") |

**Restablecimiento de fábrica:** desde la app (comando `FACTORY_RESET`) o borrando la flash con `pio run -e esp32dev -t erase` seguido de una nueva carga.

## 11. Estado y límites conocidos

- Versión del firmware: **1.0.0** · Protocolo BLE: **v1** (se reporta en `DEVICE_INFO`).
- Primer código del prototipo: **aún no probado en un vehículo en movimiento**; los umbrales por defecto (3,5 g, 300 °/s, 65°/2 s) son valores iniciales a ajustar con pruebas de campo.
- Marcados *por confirmar* en `config.h`: pines según el cableado real, divisor de batería y frecuencia de CPU (80 MHz).
- No implementados: lectura de la señal de carga (`PIN_CHRG`) y uso del LED de estado.
- `preTriggerSamples` en los metadatos de la traza siempre indica 600, aunque el anillo tenga menos muestras (solo ocurre si el evento llega en los primeros 3 s tras el arranque).
- `ACK_EVENT` modifica la cola de eventos desde el contexto del host BLE, mientras que el resto de la cola la maneja `bleServerTask`: conviene serializar ese acceso.
- Solo se conservan las 2 últimas trazas, en RAM.
- Este repositorio no incluye la documentación de diseño (`docs/03` protocolo, `docs/11` firmware) a la que remiten los comentarios del código.

## 12. Estructura del repositorio

```
include/config.h           Pines, constantes y umbrales por defecto
lib/protocol/protocol.h    Tramas BLE empaquetadas (C++ puro, compartido con las pruebas)
lib/detection/             Detector de candidatos y filtro complementario (C++ puro)
lib/ringbuffer/            Anillo circular de muestras
src/main.cpp               Arranque y creación de tareas
src/imu_task.cpp           IMU, detector, caja negra y comandos
src/ble_server.cpp         Servidor GATT (NimBLE)
src/gps_task.cpp           Lectura NMEA del NEO-6M
src/power_task.cpp         Batería
src/settings.cpp           Configuración persistente (NVS)
test/                      Pruebas unitarias nativas (Unity)
platformio.ini             Entornos: esp32dev (placa) y native (pruebas)
```
