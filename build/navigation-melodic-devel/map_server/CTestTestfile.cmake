# CMake generated Testfile for 
# Source directory: /home/lwt/jackal_ws/src/navigation-melodic-devel/map_server
# Build directory: /home/lwt/jackal_ws/build/navigation-melodic-devel/map_server
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_map_server_gtest_map_server_utest "/home/lwt/jackal_ws/build/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/lwt/jackal_ws/build/test_results/map_server/gtest-map_server_utest.xml" "--return-code" "/home/lwt/jackal_ws/devel/lib/map_server/map_server_utest --gtest_output=xml:/home/lwt/jackal_ws/build/test_results/map_server/gtest-map_server_utest.xml")
add_test(_ctest_map_server_rostest_test_rtest.xml "/home/lwt/jackal_ws/build/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/lwt/jackal_ws/build/test_results/map_server/rostest-test_rtest.xml" "--return-code" "/usr/bin/python2 /opt/ros/melodic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/lwt/jackal_ws/src/navigation-melodic-devel/map_server --package=map_server --results-filename test_rtest.xml --results-base-dir \"/home/lwt/jackal_ws/build/test_results\" /home/lwt/jackal_ws/src/navigation-melodic-devel/map_server/test/rtest.xml ")
