#sudo ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen - If Eigen library is not linked!

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O REQUIRED )

FIND_PACKAGE( CSparse REQUIRED )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

ADD_LIBRARY( optimizer_eh OptimizerE.cc )
TARGET_LINK_LIBRARIES( 
    optimizer_eh
    g2o_core 
    g2o_types_slam3d 
    g2o_solver_csparse 
    g2o_stuff 
    g2o_csparse_extension 
    ${CSPARSE_LIBRARY}
)

ADD_EXECUTABLE( OptimizationCCNY OptimizationCCNY.cc)
TARGET_LINK_LIBRARIES( OptimizationCCNY
    optimizer_eh
)

