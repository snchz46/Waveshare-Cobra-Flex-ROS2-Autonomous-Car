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
| `--density touch` | Modo tactil para la pantalla del coche (ver [Pantalla tactil](#pantalla-tactil)) |
| `--scale F` | Factor de tamano de la densidad (tactil: 1.25; escritorio: 1.0) |
| `--fullscreen` | Pantalla completa; F11 la alterna y el boton **Quit** cierra la ventana |

`COBRAFLEX_GUI_DENSITY` y `COBRAFLEX_GUI_SCALE` dan el valor por defecto de
`--density` y `--scale`, por ejemplo en un acceso directo de la Jetson.

## Pestanas

| Pestana | Contenido |
| --- | --- |
| **Launches** | Una tarjeta por launch file: descripcion, estado, PID, tiempo en marcha. La flecha despliega el comando `ros2 launch` equivalente, los argumentos y los presets. El panel inferior muestra la salida del launch seleccionado, filtrable por nivel y texto; **Hide** lo pliega y el boton **Logs** de una tarjeta lo vuelve a abrir |
| **ROS Graph** | Nodos y topicos del grafo en vivo. Un clic sobre un elemento resalta sus conexiones y muestra publicadores, suscriptores, tipos y los comandos `ros2 node info`, `ros2 topic echo` y `ros2 topic hz` correspondientes. Los botones −, + y **Fit** de la esquina del dibujo ajustan el zoom, igual que la rueda y el doble clic |
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

- La ayuda declarada de cada argumento aparece bajo su campo.
- Solo se pasan como `clave:=valor` los argumentos modificados (marca `~`).
- Un argumento sin `default_value` es obligatorio; el launch no arranca
  mientras este vacio.
- Un valor por defecto calculado (una sustitucion o una variable) no se
  conoce de antemano. El campo aparece vacio, con la expresion como pista;
  vacio significa que se aplica el valor del launch file.

## Metadatos y presets

[`config/launches.yaml`](config/launches.yaml) aporta descripcion, etiquetas
de filtro (`Sim`, `Hardware`, `RL`, `Lab`, `Tools`), favoritos y presets de
los launch files del repositorio, y `moves_robot` para los que mueven el coche
por si solos (`cobraflex_automatic`, `cobraflex_lane_keeper`,
`deploy_cobraflex`). Sin entrada, la descripcion es la primera frase del
docstring o del primer comentario, y la etiqueta `Other`. Los metadatos solo
refinan argumentos declarados (opciones, ayuda y `step` de los botones − y +);
una entrada desfasada no puede anadir un argumento que el fichero no declara.

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
| `Ctrl+K` | Paleta de comandos: lanzar o detener por nombre (tambien el boton **Search**) |
| `/` | Buscar en la lista de launches |
| `Ctrl+R` / `Ctrl+E` | Lanzar / detener el launch seleccionado |
| `Ctrl+1` .. `Ctrl+3` | Cambiar de pestana |
| `F11` | Pantalla completa |

## Aspecto

Colores y fuentes estan en [`theme.py`](cobraflex_launch_gui/theme.py) y son
los mismos en ambas densidades. La paleta procede de los materiales del coche
con la carroceria V4.1: el gris de la tapa de PLA es el fondo, el blanco de la
carcasa los paneles, el negro de ruedas y sensores el texto y la seleccion, y
el gris del chasis de aluminio el texto secundario. `style.qss` es una
plantilla que lee los colores como `${c_*}` y las fuentes como `${font_ui}` y
`${font_mono}`; el grafo, el panel de logs y el HTML de los widgets leen los
mismos valores.

| Color | Uso |
| --- | --- |
| Rojo senal (`c_signal`) | Acento unico: pestana activa, **Stop**, **Stop all**, fallos |
| Verde (`c_go`) | **Run** y **Retry** |
| Negro (`c_tyre`) | Texto, seleccion, filtros activos, paneles de comando y de logs |
| Ambar (`c_amber`) | Avisos: modo simulado, `ROS_DOMAIN_ID`, parada en curso, topicos sin publicador o sin suscriptor, transformadas sin actualizar |

Ningun otro elemento lleva color, y cada estado lleva ademas una palabra
(`running`, `stopping…`, `failed`), de modo que la ventana se lee sin vision
del color. El texto mantiene un contraste minimo de 4,5:1 sobre el fondo y
sobre los paneles.

Los iconos (Run, Stop, Restart, Logs, busqueda, favoritos) se dibujan con
QPainter en [`icons.py`](cobraflex_launch_gui/icons.py). Los simbolos Unicode
que sustituyen cambian de forma con las fuentes instaladas y difieren entre el
PC y la Jetson. Un launch en marcha se indica con un tramo de pista negra cuya
linea discontinua avanza; durante la parada la linea queda fija y en ambar.

La fuente de la interfaz es Atkinson Hyperlegible (Braille Institute, licencia
OFL), disenada para baja vision, que distingue I, l y 1. Sus ficheros no forman
parte del repositorio: los `.ttf` u `.otf` copiados en
`cobraflex_launch_gui/fonts/` se registran al arrancar la ventana, con el mismo
efecto que la fuente instalada en el sistema. En su ausencia se usa Ubuntu,
presente en el escritorio de los PC del laboratorio y de la Jetson.

## Pantalla tactil

`--density touch` adapta la ventana a la pantalla tactil del coche, una
Joy-IT RB-LCD7-2: 7", 1024x600 px sobre 154,21 x 85,92 mm, unos 0,15 mm por
pixel. Los tamanos de `style.qss` estan en pixeles independientes de la
densidad (dp) y `density.py` los convierte: la densidad tactil multiplica por
1,25, de modo que un control de 50 dp mide unos 9 mm, el minimo habitual para
un dedo, y una ventana de 820x480 dp ocupa la pantalla entera. El factor se
aplica dentro de la ventana y no con `QT_SCALE_FACTOR`, que con valores
fraccionarios dibuja lineas de un pixel desiguales en Qt 5.

```bash
ros2 run cobraflex_launch_gui launch_gui --density touch --fullscreen
```

Sin `--fullscreen`, la densidad tactil abre la ventana maximizada.

| Cambio | Motivo |
| --- | --- |
| Controles de 44 a 50 dp y texto de 12 a 22 dp | Tamano tactil. `style_touch.qss` fija la altura de los controles cuya altura depende del contenido en `style.qss` |
| Lista de launches a la izquierda y detalle del seleccionado a la derecha (`touch_page.py`) | Las tarjetas dejaban sitio para dos o tres launches y abrian los argumentos dentro de la tarjeta. Una fila solo selecciona; el boton de accion esta junto al nombre, y argumentos, salida y comando son pestanas del detalle |
| Filtro de seleccion unica: All, Sim, Robot, RL, Tools | Un toque muestra los launches del coche; con los chips de seleccion multiple eran cuatro |
| Argumentos: botones para un enum de hasta tres opciones y − / + para un numero | Cambiar un valor sin teclado. El paso es una unidad del ultimo decimal del valor por defecto, o `step` en `launches.yaml`; un numero de cuatro cifras o mas (baudios, puertos) sigue siendo un campo de texto |
| Presets antes que los argumentos | Fijan varios valores de un toque |
| Mantener pulsado 1 s para lanzar un launch con `moves_robot` | Un toque accidental no pone el coche en marcha. Detener sigue siendo un toque, y la paleta de comandos abre el detalle de estos launch en lugar de lanzarlos |
| Franja de entorno, cabecera y Stop all dentro de la barra de pestanas | La pantalla tiene 600 px de alto. Del entorno queda el aviso de `ROS_DOMAIN_ID` cuando vale 0 o no esta definido |
| Desplazamiento arrastrando con el dedo (`QScroller`) | Sin el, la lista solo se desplaza con la barra de scroll |
| Seleccion de una fila al soltar | Un arrastre que empieza sobre una fila no la selecciona |
| Barra del grafo sin busqueda ni botones de rqt_graph y RViz | Requieren teclado o una ventana propia |

El escritorio conserva las tarjetas: en un monitor grande muestran mas a la
vez, y los logs quedan visibles mientras se editan argumentos.

Segun el fabricante, el panel es multitactil de 5 puntos en PC y de un punto en
Raspberry Pi; en la Jetson, `xinput list` muestra como llega. El
desplazamiento usa el gesto de boton izquierdo de `QScroller`, valido en ambos
casos, y el zoom del grafo tiene botones en lugar de depender del pellizco.

Otra pantalla solo necesita otro factor, por ejemplo `--scale 1.0` para una
pantalla de 10" a 1024x600. Para escribir sin teclado fisico, Qt Virtual
Keyboard en modo escritorio no requiere cambios en la ventana: basta
`QT_IM_MODULE=qtvirtualkeyboard` con su plugin para Qt 5 instalado.

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
python3 -m cobraflex_launch_gui --mock --workspace ../.. --density touch   # como en el coche
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
