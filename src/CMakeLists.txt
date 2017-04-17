OPTION(BUILD_UNIT_TEST "Build UNIT TEST" ON)

SET(MY_PROGRAM_LIB_SOURCE
./IMU_LIB/IMU.cpp
./IMU_LIB/imu.h
./IMU_LIB/imucommon.cpp
./IMU_LIB/imucommon.h
./IMU_LIB/imuearth.cpp
./IMU_LIB/imuearth.h
./IMU_LIB/psins.cpp
./IMU_LIB/psins.h
./IMU_LIB/IMUPreintegrator.cpp
./IMU_LIB/IMUPreintegrator.h
./IMU_LIB/kalmanfilter.cpp
./IMU_LIB/kalmanfilter.h
./IMU_LIB/so3.cpp
./IMU_LIB/so3.h
)

SET(UNIT_TEST_SOURCE
./gtest/gtest-all.cc
./gtest/gtest.h
./gtest/gtest_main.cc
./unittest/imucommon_unittest.cpp
./unittest/imuearth_unittest.cpp 
./unittest/IMU_unittest.cpp      
./unittest/PSINS_unittest.cpp 
./unittest/kalmanfilter_unittest.cpp
)                                                                     
 
SET(MAIN_EXE_SOURCE                                   #!!!HERE!!!        
./IMU/main.cpp
)

SET(MY_PROGRAM_NAME IMU)                              #!!!HERE!!!    
SET(MY_PROGRAM_LIB ${MY_PROGRAM_NAME}_LIB)

ADD_LIBRARY(${MY_PROGRAM_LIB} ${MY_PROGRAM_LIB_SOURCE})

IF(BUILD_UNIT_TEST)
SET(UNIT_TEST ${MY_PROGRAM_NAME}_UNIT_TEST)
ADD_EXECUTABLE( ${UNIT_TEST} ${UNIT_TEST_SOURCE})
TARGET_LINK_LIBRARIES(${UNIT_TEST}        
${MY_PROGRAM_LIB})
ENDIF()

ADD_EXECUTABLE(${MY_PROGRAM_NAME} ${MAIN_EXE_SOURCE})
TARGET_LINK_LIBRARIES(${MY_PROGRAM_NAME}        
${MY_PROGRAM_LIB})                
#============AUTOGENERATE===========
