# cobraflex_teleop_gui

Ventana Qt para la conduccion manual del CobraFlex. Tres modos sobre el mismo
nodo:

- **Buttons**: rejilla 3x3; el robot se mueve mientras se mantiene pulsado un
  boton y se detiene al soltarlo.
- **Virtual joystick**: el robot sigue la posicion del mando y se detiene al
  soltarlo.
- **Sliders**: velocidad lineal y angular en unidades SI, mantenidas hasta el
  siguiente cambio o hasta pulsar STOP.

```bash
ros2 launch cobraflex_teleop_gui teleop_gui.launch.py
```

Argumentos: `cmd_vel_topic`, `max_linear`, `max_angular`, `ui_watchdog`.

## Paquete independiente

El paquete esta separado para mantener PyQt5 fuera de las dependencias de
`cobraflex`, que se instala en todas las maquinas del stack, incluida la
Jetson. Ningun paquete del workspace depende de este; un robot sin conduccion
manual no necesita compilarlo.

## Topico de salida

El nodo publica en `/cmd_vel`: `linear.x` en m/s y `angular.z` en rad/s, el
formato que leen `cobraflex_ros_driver` y el plugin DiffDrive.

**`/raw_action` no es un destino valido.** La cadena del safety cage usa otro
formato: el cage interpreta `linear.x` como acelerador en [-1, 1] y
`angular.z` como direccion normalizada, y `vehicle_control_node` los convierte
de nuevo a m/s y rad/s. Un valor de 0.3 en ese topico significa 30 % de
acelerador, no 0.3 m/s. Ademas, el cage decide en funcion de `/state_obs`
(desviacion lateral respecto al carril), que no aporta informacion sobre la
conduccion manual.

La proteccion en este modo la proporcionan el `cmd_timeout` del driver y el
watchdog de la interfaz descrito mas abajo.

## Limites

Los valores por defecto (0.35 m/s, 2.0 rad/s) corresponden a la envolvente de
planificacion del resto del stack: `max_vel_x` y `max_vel_theta` de
`nav2_params.yaml`. El driver limita a 0.53 m/s y 6.0 rad/s. Con los valores
por defecto, la interfaz no ofrece velocidades que el driver recortaria.

La barra de estado muestra el comando **publicado por el nodo**, no el valor
solicitado por el widget; un recorte por limite es visible en ella.

## Watchdog de la interfaz

Un bucle de eventos de Qt bloqueado no detiene el hilo de ROS. Sin proteccion
adicional, el temporizador seguiria publicando la ultima velocidad con la
ventana congelada, y el `cmd_timeout` del driver no se activaria porque siguen
llegando comandos.

Por ello la ventana emite un latido desde su propio bucle de eventos a 10 Hz.
Si el nodo no lo recibe durante mas de `ui_watchdog` segundos (0.5 por
defecto), publica velocidad cero y registra un aviso. El mecanismo y el valor
por defecto coinciden con `safe_action_timeout_s` de `vehicle_control_node`.

## Publicador unico

Ningun nodo arbitra `/cmd_vel`. Esta ventana no debe ejecutarse a la vez que el
lane keeper, Nav2 o `lidar_avoidance_node`: ambos publicarian en el mismo
topico y el robot ejecutaria la media temporal de los dos comandos. El otro
controlador se detiene antes, o la ventana se dirige a otro topico con
`cmd_vel_topic`.

## Origen

Adaptado de `axioma_teleop_gui`
(https://github.com/MrDavidAlv/Axioma_robot, Apache-2.0). Cambios respecto al
original:

- Limites y topico como parametros en lugar de constantes.
- Un unico porcentaje de acelerador que escala cada eje respecto a su propio
  limite, en lugar de un valor comun en m/s para ambos.
- Watchdog de la interfaz.
- Patron de apagado del repositorio (`ExternalShutdownException` y
  `if rclpy.ok()`).
- Signo del slider angular corregido; en el original estaba invertido solo en
  ese modo y giraba en sentido contrario al joystick.
