# Resource Index

This directory contains the `cobraflex` marker file used by `ament_index` to locate package resources at runtime. Do not remove it—`ros2` tooling relies on this path to resolve launch files, parameters, and installed assets.

Si añades recursos instalables (por ejemplo, nuevos launch files o YAML), asegúrate de que el paquete se instala correctamente con `setup.py` y que `ament_index` pueda encontrarlos desde aquí.
