#!/usr/bin/env python3

import os
import sys

from acados_solver_plugins import SolverPluginGenerator

from generate_ocp_iblos_pf import export_asv_pf_ocp


def main() -> int:
    # Create ocp
    acados_ocp = export_asv_pf_ocp()

    # Define the index maps
    x_index_map = {
        'x_e_bar': [0],
        'y_e_bar': [1],
        'psi': [2],
        'w': [3],
        'v_bar': [4],
    }
    z_index_map = {
        'x_e': [0],
        'y_e': [1],
    }
    p_index_map = {
        'Xv_bar': [0,1,2,3,4,5,6,7],
        'Eps_': [8],
        'coef': [9, 10, 11, 12, 13, 14],
    }
    u_index_map = {
        'u_ref': [0],
        'u_tar': [1],
        'r_ref': [2],
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
        plugin_class_name='IgblosAcados',
        solver_description='This solver contains the Path Following ASV model IGBLOS.',  # noqa: E501
        x_index_map=x_index_map,
        z_index_map=z_index_map,
        p_index_map=p_index_map,
        u_index_map=u_index_map,
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
