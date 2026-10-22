"""
alpha_ident.launch.py
Lanza hw_alpha_ident_node: ensayo del Paso 2 de la identificación (alpha e
I_offset; analizar con src/Identification/identify_alpha.py).

El robot sigue la trayectoria articular act1 con un PD + compensación de
gravedad en modo corriente, con la conversión NOMINAL fija en el código
(alpha = 208.5 ticks/N·m, sin fricción ni offset): no hay que editar
config/motorXM430W350T_params.yaml (del YAML solo se lee el bloque de
hardware) y no se necesita el nodo de Feedback Linearization.
Protocolo completo: src/Identification/Ident_OpenManX_XM430W350T_procedure.md

Secuencia: rampa 3 s desde la pose actual → trayectoria hasta t_run →
retorno quíntico (3 s) a la pose inicial → 0.5 s de pausa → torque off.
El retorno no se registra en el CSV. Ctrl+C o una parada de emergencia cortan
el torque en el acto, sin retorno.

ATENCIÓN: open_port es true por defecto → lanzar este archivo MUEVE el robot
real. Para un dry-run (recorre la trayectoria sin hardware, verifica límites y
corrientes previstas, no escribe CSV) pasar open_port:=false.

CSV: data/identification/alpha/hw_alpha_ident_<log_id>.csv

Ejemplos:
  # Dry-run (sin hardware)
  ros2 launch open_manipulator_x_torque_control alpha_ident.launch.py \
    log_id:=30 open_port:=false

  # Ensayo real (≈ 44 s en total)
  ros2 launch open_manipulator_x_torque_control alpha_ident.launch.py log_id:=30
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    overrides = {
        'open_port':  arg('open_port').lower() == 'true',
        'log_id':     int(arg('log_id')),
        't_run':      float(arg('t_run')),
        'gain_scale': float(arg('gain_scale')),
    }
    return [Node(
        package='open_manipulator_x_torque_control',
        executable='hw_alpha_ident_node',
        name='hw_alpha_ident_node',
        output='screen',
        parameters=[overrides],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('log_id',     default_value='30'),
        DeclareLaunchArgument('open_port',  default_value='true'),
        DeclareLaunchArgument('t_run',      default_value='40.0'),
        DeclareLaunchArgument('gain_scale', default_value='1.0'),
        OpaqueFunction(function=launch_setup),
    ])
