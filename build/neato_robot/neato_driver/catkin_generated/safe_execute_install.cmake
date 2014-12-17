execute_process(COMMAND "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/neato_robot/neato_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/neato_robot/neato_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
