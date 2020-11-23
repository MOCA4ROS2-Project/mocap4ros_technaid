cmake_minimum_required(VERSION 3.5)

project(imu_driver C)

find_package(ament_cmake REQUIRED)
find_package(dotnet_cmake_module REQUIRED)
find_package(ament_cmake_export_assemblies REQUIRED)
find_package(dotnet_cmake_module REQUIRED)
find_package(rcldotnet REQUIRED)
find_package(rcldotnet_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(technaid_sdk REQUIRED)

set(CSHARP_TARGET_FRAMEWORK "netcoreapp2.0")
find_package(DotNETExtra REQUIRED)

set(_assemblies_dep_dlls
    ${technaid_sdk_ASSEMBLIES_DLL}
    ${rcldotnet_common_ASSEMBLIES_DLL}
    ${sensor_msgs_ASSEMBLIES_DLL}
    ${geometry_msgs_ASSEMBLIES_DLL}
    ${std_msgs_ASSEMBLIES_DLL}
    ${rcldotnet_ASSEMBLIES_DLL}
)

set(CS_SOURCES
  ImuDriver.cs
)

add_dotnet_executable(imu_driver
  ImuDriver.cs
  INCLUDE_DLLS
  ${_assemblies_dep_dlls}
)

install_dotnet(imu_driver DESTINATION lib/${PROJECT_NAME}/dotnet)


ament_package()
