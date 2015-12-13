# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io )

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# octomap
FIND_PACKAGE( octomap REQUIRED )
INCLUDE_DIRECTORIES( ${OCTOMAP_INCLUDE_DIRS} )

ADD_EXECUTABLE( pcd2octomap pcd2octomap.cpp )
TARGET_LINK_LIBRARIES( pcd2octomap
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES})

ADD_EXECUTABLE( pcd2colorOctomap pcd2colorOctomap.cpp )
TARGET_LINK_LIBRARIES( pcd2colorOctomap
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES})

FIND_PACKAGE(OpenCV REQUIRED)

# 添加g2o的依赖
# 因为g2o不是常用库，要添加它的findg2o.cmake文件
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O )
# CSparse
FIND_PACKAGE( CSparse )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

ADD_EXECUTABLE( joinmap joinMap.cpp )
TARGET_LINK_LIBRARIES( joinmap
    ${OCTOMAP_LIBRARIES}
    ${OpenCV_LIBS}
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})

