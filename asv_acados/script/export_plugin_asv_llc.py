#!/usr/bin/env python3

import os
import sys

from acados_solver_plugins import SolverPluginGenerator

from generate_ocp_llc import export_asv_ocp


def main() -> int:
    # Create ocp
    acados_ocp = export_asv_ocp()

    # Define the index maps
    x_index_map = {
        'u': [0],
        'v': [1],
        'r': [2],
        'psi': [3],
        'mean': [4],
        'diff': [5],
    }
    z_index_map = {}
    p_index_map = {
        'Xu': [0,1,2,3,4,5],
        'Xv': [6,7,8,9,10,11,12,13,14,15,16,17],
        'Xr': [18,19,20,21,22,23,24,25,26,27,28,29],
        'Dz_up': [30],
        'Dz_down': [31],
        'se_u': [32],
        'se_v': [33],
        'se_r': [34],        
    }
    u_index_map = {
        'd_mean': [0],
        'd_diff': [1],
    }

    # Instantiate plugin generator
    dir_script_path = os.path.dirname(os.path.realpath(__file__))

    solver_plugin_generator = SolverPluginGenerator(
        custom_export_path=os.path.abspath(
            os.path.join(dir_script_path, os.pardir, 'src/plugins')),
        library_name='asv_acados'
    )

    solver_plugin_generator.generate_solver_plugin(
        acados_ocp,
        plugin_class_name='AsvAcadosSolver',
        solver_description='This solver contains the ASV model.',  # noqa: E501
        x_index_map=x_index_map,
        z_index_map=z_index_map,
        p_index_map=p_index_map,
        u_index_map=u_index_map,
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
