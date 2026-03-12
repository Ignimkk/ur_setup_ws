cmake_minimum_required(VERSION 3.8)
project(ur_setup_bringup)

find_package(ament_cmake REQUIRED)

# 런치, 설정, 모델 파일 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config urdf srdf rviz meshes worlds models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
