# ⚙️ Sistema de Control del Robot Axioma

> **NOTA**: Este documento describe el sistema de control REAL implementado en el robot Axioma. NO hay controladores PID personalizados en el código, se utiliza el plugin estándar de Gazebo con control cinemático directo.

## 1. Arquitectura de Control

El sistema de control del robot Axioma implementa una arquitectura simple basada en plugins de Gazebo:

```
Nivel de Navegación (Nav2)
    ↓
    /cmd_vel (Twist)
    ↓
Plugin Gazebo Diff Drive
    ↓
Aplicación directa de velocidades a ruedas
    ↓
Simulación física (Gazebo)
    ↓
Odometría (/odom) + TF
```

**IMPORTANTE**: No hay capa intermedia de control PID. El plugin de Gazebo aplica las velocidades directamente a las ruedas basándose en la cinemática diferencial.

---

## 2. Plugin de Control: Gazebo Diff Drive

### 2.1 Configuración

**Archivo**: `/home/axioma/ros2/axioma_humble_ws/src/axioma_description/models/axioma_v2/model.sdf`
**Líneas**: 427-454

```xml
<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
  <update_rate>50</update_rate>
  <num_wheel_pairs>2</num_wheel_pairs>

  <!-- Par izquierdo -->
  <left_joint>base_to_wheel1</left_joint>
  <left_joint>base_to_wheel2</left_joint>

  <!-- Par derecho -->
  <right_joint>base_to_wheel4</right_joint>
  <right_joint>base_to_wheel3</right_joint>

  <!-- Parámetros geométricos -->
  <wheel_separation>0.1725</wheel_separation>
  <wheel_diameter>0.0762</wheel_diameter>

  <!-- Límites -->
  <max_wheel_torque>20</max_wheel_torque>
  <max_wheel_acceleration>1.0</max_wheel_acceleration>

  <!-- Odometría -->
  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
  <odometry_frame>odom</odometry_frame>
  <robot_base_frame>base_link</robot_base_frame>
</plugin>
```

### 2.2 Funcionamiento del Plugin

**Input**: Tópico `/cmd_vel` (tipo `geometry_msgs/Twist`)
```
linear.x: velocidad lineal deseada [m/s]
angular.z: velocidad angular deseada [rad/s]
```

**Procesamiento**:
1. Lee comando de velocidad del tópico `/cmd_vel`
2. Aplica cinemática inversa (ver `cinematica.md`):
   - $\\omega_L = \\frac{v - \\omega \\cdot W/2}{r}$
   - $\\omega_R = \\frac{v + \\omega \\cdot W/2}{r}$
3. Aplica velocidades angulares a las 4 ruedas (pares sincronizados)
4. Calcula odometría desde encoders virtuales
5. Publica `/odom` y TF `odom → base_link`

**Output**:
- Tópico `/odom` (tipo `nav_msgs/Odometry`)
- Transformada TF: `odom → base_link`
- Estado de joints (via `joint_state_publisher`)

---

## 3. Parámetros de Control

### 3.1 Frecuencia de Actualización

**✅ Valor REAL**:
```
update_rate: 50 Hz
```
**Fuente**: `model.sdf:439`

Periodo de control: $\\Delta t = \\frac{1}{50} = 0.02$ s

### 3.2 Límites de Torque y Aceleración

**✅ Valores REALES de `model.sdf:446-447`**:

```
max_wheel_torque: 20 N·m
max_wheel_acceleration: 1.0 m/s²
```

**Torque máximo por rueda**: 20 N·m

**Fuerza tractiva máxima** (por rueda):
$$
F_{max} = \\frac{\\tau_{max}}{r} = \\frac{20}{0.0381} = 524.9 \\text{ N}
$$

**Fuerza tractiva total** (4 ruedas):
$$
F_{total} = 4 \\times 524.9 = 2099.6 \\text{ N}
$$

**Aceleración máxima teórica**:
$$
a_{max,theory} = \\frac{F_{total}}{m} = \\frac{2099.6}{5.525} = 380.0 \\text{ m/s}^2
$$

**NOTA**: En práctica, la aceleración está limitada por:
1. Plugin: `max_wheel_acceleration = 1.0 m/s²`
2. Nav2: `acc_lim_x = 2.5 m/s²` (`nav2_params.yaml:129`)
3. Fricción: $a_{friction} = \\mu \\cdot g = 1.0 \\times 9.81 = 9.81 \\text{ m/s}^2$

El límite efectivo es **1.0 m/s²** (más conservador).

---

## 4. Sistema de Navegación Nav2

### 4.1 Controlador Local: DWB

**Archivo**: `nav2_params.yaml` líneas 108-170
**Plugin**: `dwb_core::DWBLocalPlanner`

#### Límites de Velocidad

**✅ Valores REALES**:
```yaml
max_vel_x: 0.26 m/s              # línea 120
min_vel_x: 0.0 m/s               # línea 121
max_vel_theta: 1.0 rad/s         # línea 122
min_speed_xy: 0.0 m/s            # línea 124
max_speed_xy: 0.26 m/s           # línea 125
```

**Comando enviado a `/cmd_vel`**:
$$
\\begin{aligned}
v &\\in [0, 0.26] \\text{ m/s} \\\\
\\omega &\\in [-1.0, 1.0] \\text{ rad/s}
\\end{aligned}
$$

#### Límites de Aceleración

**✅ Valores REALES**:
```yaml
acc_lim_x: 2.5 m/s²              # línea 129
acc_lim_theta: 3.2 rad/s²        # línea 130
decel_lim_x: -2.5 m/s²           # línea 131
decel_lim_theta: -3.2 rad/s²     # línea 132
```

#### Parámetros de Muestreo

**✅ Valores REALES**:
```yaml
vx_samples: 20                   # línea 133
vy_samples: 5                    # línea 134 (no usado en diff drive)
vtheta_samples: 20               # línea 135
sim_time: 1.7 s                  # línea 136
```

El DWB genera una ventana dinámica de trayectorias:
- 20 velocidades lineales entre $[v_{min}, v_{max}]$
- 20 velocidades angulares entre $[-\\omega_{max}, \\omega_{max}]$
- Simula cada trayectoria por 1.7 segundos hacia adelante
- Selecciona la mejor trayectoria según funciones de costo

#### Funciones de Costo (Critics)

**✅ Configuración REAL** (`nav2_params.yaml:137-154`):

```yaml
critics: ["RotateToGoal", "Oscillation", "BaseObstacle",
          "GoalAlign", "PathAlign", "PathDist", "GoalDist"]

Pesos:
  BaseObstacle.scale: 0.02       # Evitar obstáculos
  PathAlign.scale: 32.0          # Alinearse con path global
  GoalAlign.scale: 24.0          # Alinearse con goal
  PathDist.scale: 32.0           # Proximidad al path
  GoalDist.scale: 24.0           # Proximidad al goal
  RotateToGoal.scale: 32.0       # Rotación hacia goal
```

Función de costo total:
$$
J_{total} = \\sum_{i} w_i \\cdot J_i
$$

---

## 5. Velocity Smoother

**Archivo**: `nav2_params.yaml` líneas 305-318
**Plugin**: `nav2_velocity_smoother::VelocitySmoother`

### 5.1 Configuración

**✅ Valores REALES**:
```yaml
smoothing_frequency: 20.0 Hz
feedback: "OPEN_LOOP"

max_velocity: [0.26, 0.0, 1.0]     # [vx, vy, ω]
min_velocity: [-0.26, 0.0, -1.0]

max_accel: [2.5, 0.0, 3.2]         # [ax, ay, α]
max_decel: [-2.5, 0.0, -3.2]
```

### 5.2 Filtro de Suavizado

El Velocity Smoother aplica restricciones de aceleración a los comandos de velocidad:

Para cada eje (x, θ):
$$
v_{cmd,k} = \\text{clip}\\left(v_{des}, v_{k-1} - a_{max}\\Delta t, v_{k-1} + a_{max}\\Delta t\\right)
$$

Donde:
- $v_{des}$: velocidad deseada del controlador
- $v_{k-1}$: velocidad anterior
- $a_{max}$: aceleración máxima
- $\\Delta t = 0.05$ s (20 Hz)

---

## 6. Localización: AMCL

**Archivo**: `nav2_params.yaml` líneas 6-50
**Plugin**: `nav2_amcl::AmclNode`

### 6.1 Modelo de Movimiento

**✅ Configuración REAL**:
```yaml
robot_model_type: "nav2_amcl::DifferentialMotionModel"

Parámetros del modelo:
  alpha1: 0.05    # Rotación por rotación
  alpha2: 0.05    # Rotación por traslación
  alpha3: 0.05    # Traslación por traslación
  alpha4: 0.05    # Traslación por rotación
  alpha5: 0.05    # Ruido adicional
```

### 6.2 Modelo de Ruido Odométrico

El modelo diferencial asume que el error en odometría es proporcional a los movimientos:

**Error en rotación**:
$$
\\sigma_{\\theta_1}^2 = \\alpha_1 \\cdot |\\Delta\\theta| + \\alpha_2 \\cdot \\sqrt{\\Delta x^2 + \\Delta y^2}
$$

**Error en traslación**:
$$
\\sigma_{trans}^2 = \\alpha_3 \\cdot \\sqrt{\\Delta x^2 + \\Delta y^2} + \\alpha_4 \\cdot |\\Delta\\theta|
$$

**Error en rotación final**:
$$
\\sigma_{\\theta_2}^2 = \\alpha_1 \\cdot |\\Delta\\theta| + \\alpha_2 \\cdot \\sqrt{\\Delta x^2 + \\Delta y^2}
$$

---

## 7. Diagrama de Flujo de Control

### 7.1 Navegación Autónoma

```
Goal (2D Nav Goal en RViz)
    ↓
Nav2 BT Navigator
    ↓
Global Planner (NavFn) → Path global
    ↓
Local Planner (DWB) → Trayectoria local óptima
    ↓
Velocity Smoother → Suavizado de aceleración
    ↓
/cmd_vel (Twist)
    ↓
Gazebo Diff Drive Plugin → Cinemática inversa
    ↓
4 Ruedas (2 pares sincronizados)
    ↓
Gazebo Physics → Simulación dinámica
    ↓
/odom + TF → Odometría
    ↓
AMCL → Localización con filtro de partículas
    ↓
/amcl_pose → Pose estimada en mapa
```

### 7.2 Teleoperation Manual

```
Joystick (Xbox controller) / Teclado
    ↓
joy_node / teleop_twist_keyboard
    ↓
teleop_twist_joy
  - scale_linear.x: 0.5 m/s
  - scale_angular.yaw: 2.0 rad/s
    ↓
/cmd_vel (Twist)
    ↓
[Mismo flujo que navegación]
```

**Fuente**: `slam_bringup.launch.py:99-107`

---

## 8. Análisis de Rendimiento

### 8.1 Tiempo de Respuesta

**Frecuencias del sistema**:
- Plugin diff_drive: 50 Hz (20 ms)
- Nav2 DWB controller: 20 Hz (50 ms)
- Velocity smoother: 20 Hz (50 ms)
- AMCL: Variable (triggered by odometry updates)

**Latencia total estimada**: ~70-100 ms desde comando hasta ejecución

### 8.2 Precisión de Odometría

La odometría en simulación es **perfecta** (sin ruido). En el robot real, los errores provendrían de:

1. **Deslizamiento de ruedas** (skid-steering inherente)
2. **Resolución de encoders**: 1000 PPR (mencionado en README, no implementado en simulación)
3. **Variación en diámetro de ruedas**

**Deriva esperada** (basado en parámetros AMCL):
- Traslacional: ~5% por metro recorrido
- Rotacional: ~5% por radián girado

---

## 9. Limitaciones del Sistema

### 9.1 No Hay Control PID Real

El sistema NO implementa control de seguimiento de velocidad con PID. El plugin de Gazebo aplica velocidades directamente (control cinemático perfecto).

**Implicación**: En el robot real se necesitaría:
- Controlador PID para cada motor
- Medición de velocidad con encoders
- Compensación de deslizamiento

### 9.2 Saturación de Velocidades

El sistema tiene múltiples capas de saturación:

1. **DWB Controller**: Limita a 0.26 m/s y 1.0 rad/s
2. **Velocity Smoother**: Aplica restricciones de aceleración
3. **Plugin Gazebo**: Limita torque (20 N·m) y aceleración (1.0 m/s²)
4. **Física de Gazebo**: Fricción y deslizamiento

---

## 10. Referencias

**Implementación**:
- Plugin Gazebo: `model.sdf:427-454`
- Controlador Nav2: `nav2_params.yaml:108-170`
- Velocity Smoother: `nav2_params.yaml:305-318`
- AMCL: `nav2_params.yaml:6-50`

**Documentación externa**:
- [Gazebo Diff Drive](http://gazebosim.org/tutorials?tut=ros2_diff_drive)
- [Nav2 DWB Controller](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html)
- [AMCL Documentation](https://navigation.ros.org/configuration/packages/configuring-amcl.html)

---

**Autor**: Samuel Sanchez
**Fecha**: 2026
**Versión**: 0.2.0
