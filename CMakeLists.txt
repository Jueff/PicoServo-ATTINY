   cmake_minimum_required(VERSION 3.13)
   project(PicoServo NONE)

   add_custom_target(
       build_with_cmd
       COMMAND ${CMAKE_COMMAND} -E echo "Starte compile.cmd..."
       COMMAND compile.cmd
       WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
   )

   # Dummy-Hauptziel, das von build_with_cmd abhängt
   add_custom_target(main ALL)
   add_dependencies(main build_with_cmd)