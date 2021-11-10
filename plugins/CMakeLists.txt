set(PLUGINS_FOLDERS "rtph264pass")

foreach(FOLDER ${PLUGINS_FOLDERS})
   add_subdirectory("${FOLDER}")
endforeach()
