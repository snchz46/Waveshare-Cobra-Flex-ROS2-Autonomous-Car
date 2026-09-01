# cobraflex_teleop_gui

Ventana Qt para conducir el CobraFlex a mano. Tres modos sobre el mismo nodo:

- **Buttons** - rejilla 3x3, conduce mientras mantienes pulsado y para al soltar.
- **Virtual joystick** - arrastras el mando, sueltas y para.
- **Sliders** - fijas velocidad lineal y angular en SI y se mantienen hasta que
  las cambies o pulses STOP.

```bash
ros2 launch cobraflex_teleop_gui teleop_gui.launch.py
```

Argumentos: `cmd_vel_topic`, `max_linear`, `max_angular`, `ui_watchdog`.

## Por que es un paquete aparte

Para que PyQt5 no entre en las dependencias de `cobraflex`, que es el paquete
que necesita cualquier maquina del stack, incluida la Jetson. Nada del
workspace depende de este, asi que un robot al que nunca se conduce a mano
simplemente no lo construye.

## Publica en cmd_vel, y eso es lo correcto

`linear.x` en m/s y `angular.z` en rad/s, que es lo que leen
`cobraflex_ros_driver` y el plugin DiffDrive.

**No lo apuntes a `/raw_action` pensando que asi pasa por el safety cage.** Esa
cadena habla otro contrato: el cage toma `linear.x` como acelerador en [-1, 1]
y `angular.z` como direccion normalizada, y es `vehicle_control_node` quien
convierte eso de vuelta a m/s y rad/s. Un 0.3 ahi significa 30 % de acelerador,
no 0.3 m/s. Ademas el cage decide sobre `/state_obs` -- desviacion lateral
respecto al carril -- que no dice nada de una persona conduciendo a mano.

Lo que si te protege aqui es el `cmd_timeout` del driver, y el watchdog de UI
que se describe abajo.

## Limites

Los valores por defecto (0.35 m/s, 2.0 rad/s) son la envolvente dentro de la
que planifica el resto del stack: `max_vel_x` y `max_vel_theta` de
`nav2_params.yaml`. El driver corta mas arriba, en 0.53 m/s y 6.0 rad/s. Se
eligieron los pequenos a proposito: un slider que promete una velocidad que el
robot luego recorta en silencio es peor que uno que se queda dentro de lo que
todo lo demas ya da por bueno.

La barra de estado muestra lo que **publica el nodo**, no lo que pide el
widget. Si un limite recorta, se ve ahi.

## Watchdog de UI

Un bucle de eventos de Qt colgado no para el hilo de ROS. Sin nada mas, el
temporizador seguiria republicando la ultima velocidad para siempre con la
ventana congelada, y el `cmd_timeout` del driver nunca saltaria porque
comandos si estan llegando.

Asi que la ventana hace de latido desde dentro de su propio bucle de eventos, a
10 Hz, y si el nodo pasa mas de `ui_watchdog` segundos (0.5 por defecto) sin
recibirlo, comanda cero y avisa por el log. Es la misma idea, y el mismo valor
por defecto, que el `safe_action_timeout_s` de `vehicle_control_node`.

## Un solo comandante

Nada arbitra `/cmd_vel`. No lances esto a la vez que el lane keeper, Nav2 o
`lidar_avoidance_node`: los dos publicarian sobre el mismo topico y el robot
haria la media temporal de ambos. Para el otro primero, o mueve esta ventana a
un topico distinto con `cmd_vel_topic`.

## Origen

Adaptado de `axioma_teleop_gui`
(https://github.com/MrDavidAlv/Axioma_robot, Apache-2.0). Cambios respecto al
original: limites y topico como parametros en vez de constantes, un solo
porcentaje de acelerador escalando cada eje contra su propio limite en vez de
una cifra unica en m/s para los dos, el watchdog de UI, el patron de apagado de
la casa (`ExternalShutdownException` y `if rclpy.ok()`), y el signo del slider
angular, que en el original iba negado solo en ese modo y por tanto giraba al
reves que su propio joystick.
