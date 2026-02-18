# 📏 Parámetros Físicos del Robot Axioma

> **NOTA IMPORTANTE**: Este documento contiene SOLO valores REALES extraídos directamente del código fuente. Los parámetros están marcados con ✅ y la fuente exacta (archivo y línea).

---

## 1. Geometría del Robot

### 1.1 Dimensiones de Ruedas

| Parámetro | Símbolo | Valor | Fuente |
|-----------|---------|-------|--------|
| Radio de rueda | $r$ | 0.0381 m | ✅ `model.sdf:72,149,226,303` |
| Diámetro de rueda | $d$ | 0.0762 m | ✅ `model.sdf:443` |
| Ancho de rueda | $w$ | 0.03 m | ✅ `model.sdf:73,150,227,304` |

### 1.2 Dimensiones del Chasis

| Parámetro | Símbolo | Valor | Fuente |
|-----------|---------|-------|--------|
| Separación de ruedas | $W$ | 0.1725 m | ✅ `model.sdf:441` |
| Distancia entre ejes | $L$ | 0.1356 m | ✅ Calculado: \|0.0644-(-0.0712)\| |
| Largo estimado | - | ~0.14 m | Estimado de posiciones |
| Ancho estimado | - | ~0.14 m | Estimado de posiciones |

**NOTA**: El chasis usa meshes personalizados (chasis.dae), no hay dimensiones de box explícitas.

### 1.3 Posiciones de Ruedas

**Coordenadas en Frame `base_link`** (✅ `model.sdf`):

| Rueda | Posición $(x, y, z)$ [m] | Joint | Línea |
|-------|---------------------------|-------|-------|
| Wheel 1 (FL) | (0.06442, 0.07385, 0.04068) | `base_to_wheel1` | 38 |
| Wheel 2 (RL) | (-0.071224, 0.07385, 0.04068) | `base_to_wheel2` | 115 |
| Wheel 3 (RR) | (-0.071224, -0.0625, 0.04068) | `base_to_wheel3` | 192 |
| Wheel 4 (FR) | (0.064423, -0.0625, 0.04068) | `base_to_wheel4` | 269 |

---

## 2. Propiedades Inerciales

### 2.1 Masa Total del Robot

| Componente | Cantidad | Masa Unitaria | Masa Total |
|------------|----------|---------------|------------|
| Base Link (chasis) | 1 | 5.0 kg | 5.0 kg |
| Ruedas (wheel_1...4) | 4 | 0.1 kg | 0.4 kg |
| LiDAR (base_scan) | 1 | 0.125 kg | 0.125 kg |
| **TOTAL** | - | - | **5.525 kg** |

**Fuentes**: `model.sdf:8, 55, 132, 209, 286, 348`

### 2.2 Tensor de Inercia del Chasis

**Base Link** (✅ Valores REALES de `model.sdf:6-17`):

$$
\\mathbf{I}_{base} = \\begin{bmatrix}
0.1135 & 0 & 0 \\\\
0 & 0.426 & 0 \\\\
0 & 0 & 0.5205
\\end{bmatrix} \\text{ kg·m}^2
$$

```xml
<link name="base_link">
  <mass>5.0</mass>
  <inertia>
    <ixx>0.1135</ixx>
    <iyy>0.426</iyy>
    <izz>0.5205</izz>
    <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
  </inertia>
</link>
```

### 2.3 Tensor de Inercia de Ruedas

**Cada Rueda** (✅ Valores REALES de `model.sdf:55-65, 132-142, 209-219, 286-296`):

$$
\\mathbf{I}_{wheel} = \\begin{bmatrix}
0.0029 & 0 & 0 \\\\
0 & 0.0029 & 0 \\\\
0 & 0 & 0.0056
\\end{bmatrix} \\text{ kg·m}^2, \\quad m_{wheel} = 0.1 \\text{ kg}
$$

```xml
<link name="wheel_X">
  <mass>0.1</mass>
  <inertia>
    <ixx>0.0029</ixx>
    <iyy>0.0029</iyy>
    <izz>0.0056</izz>
  </inertia>
</link>
```

### 2.4 Tensor de Inercia del LiDAR

**LiDAR** (✅ Valores REALES de `model.sdf:348-358`):

$$
\\mathbf{I}_{lidar} = \\begin{bmatrix}
0.001 & 0 & 0 \\\\
0 & 0.001 & 0 \\\\
0 & 0 & 0.001
\\end{bmatrix} \\text{ kg·m}^2, \\quad m_{lidar} = 0.125 \\text{ kg}
$$

---

## 3. Parámetros Dinámicos

### 3.1 Fricción Rueda-Suelo

**✅ Valores REALES de `model.sdf:77-96, 154-173, 231-250, 308-327`**:

```xml
<friction>
  <ode>
    <mu>1.0</mu>
    <mu2>1.0</mu2>
    <slip1>0.0</slip1>
    <slip2>0.0</slip2>
  </ode>
</friction>
```

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| $\\mu_1$ | 1.0 | Coeficiente de fricción de Coulomb direccional |
| $\\mu_2$ | 1.0 | Coeficiente de fricción de Coulomb lateral |
| slip1 | 0.0 | Deslizamiento direccional |
| slip2 | 0.0 | Deslizamiento lateral |

**Fuerza de fricción máxima**:
$$
F_{friction,max} = \\mu \\cdot F_N = 1.0 \\times \\frac{mg}{4} = 1.0 \\times \\frac{5.525 \\times 9.81}{4} = 13.55 \\text{ N por rueda}
$$

**Aceleración máxima por fricción**:
$$
a_{friction,max} = \\mu \\cdot g = 1.0 \\times 9.81 = 9.81 \\text{ m/s}^2
$$

### 3.2 Parámetros de Contacto (ODE)

**✅ Valores REALES de `model.sdf:87-96`**:

```xml
<contact>
  <ode>
    <soft_cfm>0</soft_cfm>
    <soft_erp>0.2</soft_erp>
    <kp>1e+13</kp>
    <kd>1.0</kd>
    <max_vel>0.01</max_vel>
    <min_depth>0.01</min_depth>
  </ode>
</contact>
```

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `soft_cfm` | 0 | Constraint Force Mixing (rigidez del contacto) |
| `soft_erp` | 0.2 | Error Reduction Parameter (corrección de penetración) |
| `kp` | $10^{13}$ | Rigidez del contacto (spring constant) |
| `kd` | 1.0 | Amortiguamiento del contacto (damping) |
| `max_vel` | 0.01 m/s | Velocidad máxima de penetración permitida |
| `min_depth` | 0.01 m | Profundidad mínima de contacto para activación |

### 3.3 Amortiguamiento de Joints

**NOTA**: No hay parámetros de `<damping>` o `<friction>` activos en los joints de las ruedas. Los tags están comentados en el SDF.

---

## 4. Límites Operacionales

### 4.1 Límites Cinemáticos

**✅ Valores REALES de `nav2_params.yaml`**:

| Parámetro | Valor | Fuente |
|-----------|-------|--------|
| Velocidad lineal máxima | $v_{max} = 0.26$ m/s | `nav2_params.yaml:120` |
| Velocidad angular máxima | $\\omega_{max} = 1.0$ rad/s | `nav2_params.yaml:122` |
| Aceleración lineal máxima | $a_{max} = 2.5$ m/s² | `nav2_params.yaml:129` |
| Aceleración angular máxima | $\\alpha_{max} = 3.2$ rad/s² | `nav2_params.yaml:130` |
| Desaceleración lineal | $a_{min} = -2.5$ m/s² | `nav2_params.yaml:131` |
| Desaceleración angular | $\\alpha_{min} = -3.2$ rad/s² | `nav2_params.yaml:132` |

### 4.2 Límites de Torque

**✅ Valores REALES de `model.sdf:446-447`**:

```xml
<max_wheel_torque>20</max_wheel_torque>
<max_wheel_acceleration>1.0</max_wheel_acceleration>
```

| Parámetro | Valor |
|-----------|-------|
| Torque máximo por rueda | 20 N·m |
| Aceleración máxima de rueda (plugin) | 1.0 m/s² |

### 4.3 Límites de Joints

**✅ Valores REALES de `model.sdf:43-46, 120-123, 197-200, 274-277`**:

```xml
<limit>
  <lower>-1e+16</lower>
  <upper>1e+16</upper>
</limit>
```

Las ruedas pueden girar infinitamente (tipo revolute sin límites).

---

## 5. Sensor LiDAR

### 5.1 Especificaciones

**✅ Valores REALES de `model.sdf:346-426`**:

| Parámetro | Valor | Línea |
|-----------|-------|-------|
| Posición en robot | $(0, 0, 0.15)$ m desde `base_link` | 347 |
| Tipo | Láser 2D planar | 351 |
| Modelo | `hls_lfcd_lds` | 375 |
| Muestras por barrido | 360 | 383 |
| Resolución angular | 2.0° | 384 |
| Ángulo mínimo | 0° (0 rad) | 385 |
| Ángulo máximo | 360° (6.28 rad) | 386 |
| Rango mínimo | 0.12 m | 24, 390 |
| Rango máximo | 3.5 m | 391 |
| Frecuencia | 20 Hz | 379 |
| Ruido (media) | 0.0 | 395 |
| Ruido (stddev) | 0.01 | 396 |

**Frame**: `base_scan` (línea 413)
**Tópico**: `/scan` (línea 403)

### 5.2 Transformada del LiDAR

Transformada de `base_link` → `base_scan`:

$$
\\mathbf{T}_{scan}^{base} = \\begin{bmatrix}
1 & 0 & 0 & 0 \\\\
0 & 1 & 0 & 0 \\\\
0 & 0 & 1 & 0.15 \\\\
0 & 0 & 0 & 1
\\end{bmatrix}
$$

El LiDAR está montado **15 cm arriba** del origen de `base_link`.

---

## 6. Parámetros de Navegación

### 6.1 Costmaps

**Local Costmap** (✅ `nav2_params.yaml:171-209`):

```yaml
update_frequency: 5.0 Hz
publish_frequency: 2.0 Hz
width: 3 m
height: 3 m
resolution: 0.05 m/celda
robot_radius: 0.15 m
inflation_radius: 0.55 m
cost_scaling_factor: 3.0
```

**Global Costmap** (✅ `nav2_params.yaml:211-244`):

```yaml
update_frequency: 1.0 Hz
publish_frequency: 1.0 Hz
resolution: 0.05 m/celda
robot_radius: 0.15 m
inflation_radius: 0.55 m
cost_scaling_factor: 3.0
```

### 6.2 Tolerancias de Goal

**✅ Valores REALES de `nav2_params.yaml:163-164`**:

```yaml
xy_goal_tolerance: 0.15 m
yaw_goal_tolerance: 0.25 rad (≈14.3°)
```

---

## 7. Parámetros de SLAM

### 7.1 SLAM Toolbox

**✅ Valores REALES de `slam_params.yaml`**:

```yaml
resolution: 0.05 m/celda
max_laser_range: 3.5 m
minimum_time_interval: 0.5 s
minimum_travel_distance: 0.5 m
minimum_travel_heading: 0.5 rad (≈28.6°)

Loop closure:
  do_loop_closing: true
  loop_search_maximum_distance: 3.0 m
  loop_match_minimum_chain_size: 10
```

### 7.2 Mapa Existente

**✅ Valores REALES de `maps/mapa.yaml`**:

```yaml
Imagen: mapa.pgm
Dimensiones: 215 × 231 píxeles
Resolución: 0.05 m/píxel
Origin: [-5.69, -5.14, 0] metros
occupied_thresh: 0.65
free_thresh: 0.25
```

**Área del mapa**:
- Ancho: $215 \\times 0.05 = 10.75$ m
- Alto: $231 \\times 0.05 = 11.55$ m
- Área total: $\\approx 124$ m²

---

## 8. Cálculos Derivados

### 8.1 Desempeño Cinemático

**Velocidad angular de rueda a máxima velocidad lineal**:
$$
\\omega_{wheel} = \\frac{v_{max}}{r} = \\frac{0.26}{0.0381} = 6.82 \\text{ rad/s} = 65.1 \\text{ RPM}
$$

**Velocidad angular de rueda en rotación pura**:
$$
\\omega_{wheel,rot} = \\frac{\\omega_{max} \\cdot W}{2r} = \\frac{1.0 \\times 0.1725}{2 \\times 0.0381} = 2.26 \\text{ rad/s} = 21.6 \\text{ RPM}
$$

### 8.2 Distancia de Frenado

Desde velocidad máxima con desaceleración máxima:
$$
d_{brake} = \\frac{v_{max}^2}{2 \\cdot |a_{min}|} = \\frac{0.26^2}{2 \\times 2.5} = 0.0135 \\text{ m} = 1.35 \\text{ cm}
$$

### 8.3 Radio de Giro Mínimo

En rotación pura ($v = 0$):
$$
R_{min} = 0 \\text{ m (giro de punto cero)}
$$

A velocidad máxima con rotación máxima:
$$
R = \\frac{v_{max}}{\\omega_{max}} = \\frac{0.26}{1.0} = 0.26 \\text{ m}
$$

---

## 9. Tabla Resumen de Parámetros

### 9.1 Parámetros Geométricos

```python
PARAMS_GEOMETRY = {
    'wheel_radius': 0.0381,      # m (model.sdf:72)
    'wheel_separation': 0.1725,  # m (model.sdf:441)
    'wheel_base': 0.1356,        # m (calculado)
    'total_mass': 5.525          # kg (suma de masas)
}
```

### 9.2 Parámetros Cinemáticos

```python
PARAMS_KINEMATICS = {
    'max_linear_velocity': 0.26,      # m/s (nav2_params:120)
    'max_angular_velocity': 1.0,      # rad/s (nav2_params:122)
    'max_linear_acceleration': 2.5,   # m/s² (nav2_params:129)
    'max_angular_acceleration': 3.2,  # rad/s² (nav2_params:130)
    'max_wheel_torque': 20.0          # N·m (model.sdf:446)
}
```

### 9.3 Parámetros de Control

```python
PARAMS_CONTROL = {
    'gazebo_update_rate': 50.0,       # Hz (model.sdf:439)
    'nav2_controller_freq': 20.0,     # Hz (nav2_params:114)
    'velocity_smoother_freq': 20.0,   # Hz (nav2_params:307)
    'costmap_update_freq': 5.0        # Hz (nav2_params:173)
}
```

---

## 10. Inconsistencias Detectadas

### 10.1 Masa del Base Link

**SDF vs URDF**:
- `model.sdf:8`: masa = 5.0 kg ✅ (usado en Gazebo)
- `axioma.urdf:22`: masa = 1.0 kg ⚠️ (inconsistente)

**Recomendación**: Unificar en 5.0 kg

### 10.2 Wheel Separation

**Plugin vs Geometría**:
- `model.sdf:441`: wheel_separation = 0.1725 m ✅ (usado por plugin)
- Posiciones reales: $y_{left} - y_{right} = 0.07385 - (-0.0625) = 0.13635$ m ⚠️

**Diferencia**: 3.6 cm (21% de error)

**Impacto**: Puede causar deriva en odometría durante rotaciones.

---

## 11. Referencias Cruzadas

- **Cinemática**: [cinematica.md](./cinematica.md) utiliza estos parámetros geométricos
- **Control**: [control.md](./control.md) utiliza límites y frecuencias
- **Archivos fuente**:
  - Modelo principal: `src/axioma_description/models/axioma_v2/model.sdf`
  - Parámetros Nav2: `src/axioma_navigation/config/nav2_params.yaml`
  - Parámetros SLAM: `src/axioma_navigation/config/slam_params.yaml`

---

**Autor**: Samuel Sanchez
**Fecha**: 2026
**Versión**: 0.2.0
