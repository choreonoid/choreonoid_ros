set(pkgconfig_dir ${PROJECT_BINARY_DIR}/lib/pkgconfig)

foreach(file choreonoid-ros-plugin.pc)
  configure_file(${file}.in ${pkgconfig_dir}/${file} @ONLY)
  if(INSTALL_SDK)
    install(FILES ${pkgconfig_dir}/${file} DESTINATION lib/pkgconfig)
  endif()
endforeach()
