# cobraflex_launch_gui

Ventana Qt para lanzar y observar el stack: lista los launch files de todos
los workspaces activos, edita sus argumentos, los arranca y detiene con un
panel de logs por launch, y dibuja en vivo el grafo ROS (nodos y topicos) y
el arbol TF.

```bash
ros2 run cobraflex_launch_gui launch_gui
```

| Opcion | Efecto |
| --- | --- |
| `--all` | Incluye tambien los paquetes de `/opt/ros` |
| `--workspace RUTA` | Busca en `RUTA/src` en lugar del indice de ament |
| `--mock` | Simula ejecuciones, grafo y TF; no arranca ningun proceso |

## Pestanas

| Pestana | Contenido |
| --- | --- |
| **Launches** | Una tarjeta por launch file: descripcion, estado, PID, tiempo en marcha. La flecha despliega el comando `ros2 launch` equivalente, los argumentos y los presets. El panel inferior muestra la salida del launch seleccionado, filtrable por nivel y texto |
| **ROS Graph** | Nodos y topicos del grafo en vivo. Un clic sobre un elemento resalta sus conexiones y muestra publicadores, suscriptores, tipos y los comandos `ros2 node info`, `ros2 topic echo` y `ros2 topic hz` correspondientes |
| **TF Tree** | Frames recibidos en `/tf` y `/tf_static`, con la frecuencia de cada transformada y un diagnostico al pie |

Colores del grafo: un topico en ambar no tiene publicador o no tiene
suscriptor; un nodo en rojo comparte nombre con otro nodo. En el arbol TF,
rojo indica un frame con mas de un padre y ambar una transformada sin
actualizar desde hace mas de 2 s. El diagnostico senala ademas si los frames
forman varios arboles separados.

`/clock`, `/tf` y `/tf_static` se ocultan por defecto en el grafo: en
simulacion casi todos los nodos se suscriben a `/clock`, y el dibujo se
reduce a dos nodos centrales. La casilla correspondiente los muestra.

## Descubrimiento

Los launch files se leen del indice de ament de los workspaces activos
(`<prefijo>/share/<paquete>/launch/`), sin los de `/opt/ros`. La lista
contiene por tanto exactamente los launch files que `ros2 launch` puede
ejecutar. Un overlay como `~/lab_ws` aparece en cuanto se ha compilado y
cargado con `source`; un launch file nuevo requiere `colcon build` antes de
aparecer, igual que en la linea de comandos.

## Argumentos

Los argumentos se leen del propio launch file sin ejecutarlo: el AST de un
fichero Python (`DeclareLaunchArgument`), los `<arg>` de primer nivel de un
fichero XML y las entradas `arg` de un fichero YAML.

- Solo se pasan como `clave:=valor` los argumentos modificados (marca `~`).
- Un argumento sin `default_value` es obligatorio; el launch no arranca
  mientras este vacio.
- Un valor por defecto calculado (una sustitucion o una variable) no se
  conoce de antemano. El campo aparece vacio, con la expresion como pista;
  vacio significa que se aplica el valor del launch file.

## Metadatos y presets

[`config/launches.yaml`](config/launches.yaml) aporta descripcion, etiquetas
de filtro (`Sim`, `Hardware`, `RL`, `Lab`, `Tools`), favoritos y presets de
los launch files del repositorio. Sin entrada, la descripcion es la primera
frase del docstring o del primer comentario, y la etiqueta `Other`. Los
metadatos solo refinan argumentos declarados (opciones y ayuda); una entrada
desfasada no puede anadir un argumento que el fichero no declara.

Los paquetes `cobraflex_lab` reciben la etiqueta `Lab`.

Datos por usuario, en `~/.config/cobraflex_launch_gui/`:

| Fichero | Contenido |
| --- | --- |
| `state.json` | Favoritos y presets guardados con **+ Save current** (clic derecho para borrar) |
| `launches.yaml` | Mismo formato que `config/launches.yaml`; sus entradas prevalecen |

## Parada de procesos

Cada launch arranca en una sesion propia (`setsid`). **Stop** envia SIGINT
a todo su grupo de procesos, la misma senal que Ctrl-C en un terminal. Los
procesos que abandonan el grupo (`gz sim` llama a `setsid()`) se localizan en
`/proc` y reciben la senal por separado. Si algo sigue vivo a los 8 s, la
ventana envia SIGTERM, y SIGKILL 3 s despues. `ros2 launch` escala antes por
su cuenta (SIGTERM a los 5 s); el plazo de la ventana es solo un respaldo.

Al cerrar la ventana con launches en marcha se pide confirmacion y se
detienen todos. Ctrl-C en el terminal que arranco la ventana tiene el mismo
efecto: sin ello, los launches, que estan en sesiones propias, seguirian en
marcha.

Un proceso que termina con error mientras el launch sigue activo se senala
en la tarjeta ("1 process(es) died") junto con la ultima linea de error.

## Atajos

| Tecla | Accion |
| --- | --- |
| `Ctrl+K` | Paleta de comandos: lanzar o detener por nombre |
| `/` | Buscar en la lista de launches |
| `Ctrl+R` / `Ctrl+E` | Lanzar / detener el launch seleccionado |
| `Ctrl+1` .. `Ctrl+3` | Cambiar de pestana |

## Limites

- **Atribucion de nodos.** ROS 2 no expone que proceso aloja cada nodo. El
  filtro por launch de la pestana ROS Graph usa los nombres de logger y de
  proceso de la salida del launch; un nodo que nunca escribe en el log y cuyo
  nombre difiere del de su ejecutable no se atribuye.
- **Launch files del robot.** Las tarjetas `Hardware` arrancan los
  controladores locales de la maquina donde corre la ventana. En el PC del
  laboratorio fallan por falta de puerto serie; se ejecutan en la Jetson.
- **Plataforma.** La ejecucion real requiere Linux (`setsid`, `/proc`). El
  modo `--mock` funciona en cualquier sistema.
- **Disposicion.** El grafo se ordena con Graphviz (`dot`). Sin Graphviz se
  usa un algoritmo por capas integrado, con mas cruces de aristas.

## Desarrollo sin ROS

```bash
pip install PyQt5 PyYAML
cd src/cobraflex_launch_gui
python3 -m cobraflex_launch_gui --mock --workspace ../..
```

## Paquete independiente

El paquete esta separado por el mismo motivo que `cobraflex_teleop_gui`:
mantener PyQt5 y Graphviz fuera de las dependencias de `cobraflex`, que se
instala en todas las maquinas del stack, incluida la Jetson. Ningun paquete
del workspace depende de este.

## Origen

Version integrada del launch manager de la tesis (PySide6). Cambios
respecto a aquella version:

- PyQt5, como el resto del workspace, instalable con `rosdep`.
- Descubrimiento por indice de ament en lugar de `src/*/launch`.
- Argumentos siempre leidos del launch file; antes, los metadatos los
  sustituian y estaban desfasados.
- Grafo y arbol TF nativos sobre rclpy, en lugar de paginas HTML con
  dependencias de CDN alimentadas por llamadas periodicas a la CLI `ros2`.
- Clasificacion de la salida de los nodos por nivel, parada fiable con
  reinicio, favoritos y presets persistentes.
