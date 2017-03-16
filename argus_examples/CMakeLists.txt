cmake_minimum_required(VERSION 2.8.3)
project(argus_examples)
find_package(catkin REQUIRED)

catkin_package()

# install
install(FILES
   launch/*.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
