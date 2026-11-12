# CMake generated Testfile for 
# Source directory: /home/cai/Ground_station/src/ground_station
# Build directory: /home/cai/Ground_station/build/ground_station
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_ground_station_nosetests_test "/home/cai/Ground_station/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/cai/Ground_station/build/test_results/ground_station/nosetests-test.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/cai/Ground_station/build/test_results/ground_station" "/usr/bin/nosetests3 -P --process-timeout=60 --where=/home/cai/Ground_station/src/ground_station/test --with-xunit --xunit-file=/home/cai/Ground_station/build/test_results/ground_station/nosetests-test.xml")
set_tests_properties(_ctest_ground_station_nosetests_test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/cai/Ground_station/src/ground_station/CMakeLists.txt;19;catkin_add_nosetests;/home/cai/Ground_station/src/ground_station/CMakeLists.txt;0;")
