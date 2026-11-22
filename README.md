# Robotin · Clasificador de palta Hass

Simulación en Ubuntu con ROS 2 Lyrical y Gazebo Jetty. Incluye la faja de
500 × 150 mm, dos rodillos con cunas animadas, captura cenital y un canal giratorio
inclinado 28°. La demostración distribuye tres paltas a almacén, descarte y consumo.

## Abrir la demostración

Desde la carpeta del proyecto:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-select robotin_sim
source install/local_setup.bash
ros2 launch robotin_sim sorter.launch.py
```

Gazebo y el panel **Robotin · Inspección** se abren en ventanas separadas. El
panel comienza en pausa. Pulsa **Reproducir**, o usa **Paso** para avanzar 50 ms.
**Reiniciar** devuelve los frutos al inicio y deja la máquina en pausa.

**General**, **Descarga**, **Superior** y **Perfil** cambian la vista de Gazebo.
**Ocultar tapa** deja la lente y el interior del túnel a la vista.
El panel muestra por palta el destino, el margen entre alineación y llegada al
borde, y la tolva observada. **En vivo** alterna entre el flujo continuo y el
último fotograma seleccionado. La terminal y `/robotin/result` muestran el
resultado de la secuencia y la altura aproximada del rebote después del primer
paso cerca del fondo de la tolva.

Cada lanzamiento usa su propio mundo de Gazebo y sus propios canales de
control. Si defines manualmente `GZ_PARTITION`, usa un valor distinto para
cada lanzamiento simultáneo. Los registros de inicio y errores se guardan en
`~/.ros/log`.

## Ensayos reproducibles

| Argumento | Valores |
|---|---|
| `belt_speed_mm_s` | 60, 100 o 120 |
| `diameter_mm` | 55 a 95 |
| `mass_kg` | 0,18 a 0,32. Usa punto decimal en la terminal. |
| `first_class`, `second_class`, `third_class` | 0 almacén, 1 consumo, 2 descarte. Usa `third_class:=-1` para dos frutos y también `second_class:=-1` para uno. |
| `fixed_class` | Fija el canal en la posición de una clase durante toda la prueba. |
| `headless`, `camera`, `viewer`, `paused` | `true` o `false`. Sin interfaz, la prueba arranca por defecto. |

Dos paltas consecutivas con destinos opuestos a 120 mm/s:

```bash
ros2 launch robotin_sim sorter.launch.py headless:=true camera:=false \
  belt_speed_mm_s:=120 third_class:=-1
```

Prueba de descarga con el canal fijo y frutos de 95 mm y 320 g:

```bash
ros2 launch robotin_sim sorter.launch.py headless:=true camera:=false \
  fixed_class:=2 diameter_mm:=95 mass_kg:=0.32 second_class:=-1 third_class:=-1
```

## Clasificación por imagen

La cámara simulada publica RGB de 320 × 320 en `/robotin/camera/image`.
`/robotin/camera/capture` conserva el último fotograma seleccionado.
La demostración usa clases predefinidas.

Para cargar un modelo propio, instala `ai-edge-litert` en un entorno virtual
creado con `python3 -m venv --system-site-packages .venv` y actívalo antes de
lanzar ROS. Indica el archivo con `model:=/ruta/al/modelo.tflite`. Se espera el
contrato del informe, con RGB float32 de 160 × 160 en [0, 255], normalización
dentro del grafo y tres probabilidades de salida. El controlador toma la
decisión por mínimo riesgo con la matriz del informe.

## Alcance físico

- La velocidad tangencial del rodillo determina el transporte de cada fruto.
  Las cunas animadas ayudan a inspeccionar el movimiento. Gazebo calcula la
  caída libre y los contactos en el canal y el colector.
- La biela sigue la cinemática de brazos paralelos. La simulación mide los
  ángulos y los tiempos del ciclo. El esfuerzo de la biela, el par de carga
  sobre el servo y las propiedades ópticas de una OV2640 física no se han calibrado.
