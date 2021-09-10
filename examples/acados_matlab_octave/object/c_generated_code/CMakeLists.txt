cmake_minimum_required(VERSION 3.0.2)
project(lee_controller)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


include_directories(/usr/include/eigen3)


include_directories(/home/akash/Downloads/acados/examples/acados_matlab_octave/object/../../../include)
include_directories(/home/akash/Downloads/acados/examples/acados_matlab_octave/object/../../../include/blasfeo/include)
include_directories(/home/akash/Downloads/acados/examples/acados_matlab_octave/object/../../../include/hpipm/include/)
link_directories(/home/akash/Downloads/acados/examples/acados_matlab_octave/object/../../../lib)


add_library( libacados_sim_solver_ocp_model acados_sim_solver_ocp_model.c)
add_library( acados_solver_ocp_model acados_solver_ocp_model.c)
#add_library( acados_sim_solver_ocp_model acados_sim_solver_ocp_model.c)
#add_library( libacados_sim_solver_ocp_model acados_sim_solver_ocp_model.c)

add_executable(main_ocp_model_cmake main_ocp_model.cpp ocp_model_model/ocp_model_expl_vde_forw.c ocp_model_model/ocp_model_expl_ode_fun.c)
target_link_libraries( main_ocp_model_cmake libacados_sim_solver_ocp_model acados_solver_ocp_model acados hpipm blasfeo m)


