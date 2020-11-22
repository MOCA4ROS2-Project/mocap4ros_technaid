cmake_minimum_required(VERSION 3.5)

project(technaid_sdk C)

find_package(ament_cmake REQUIRED)
find_package(dotnet_cmake_module REQUIRED)
find_package(ament_cmake_export_assemblies REQUIRED)
find_package(dotnet_cmake_module REQUIRED)

set(CSHARP_TARGET_FRAMEWORK "netcoreapp2.0")
find_package(DotNETExtra REQUIRED)

install(FILES
 "SDK\ MCS\ Studio2.dll"
  MatrixLibrary.dll
  DESTINATION lib/${PROJECT_NAME}/dotnet)


ament_export_assemblies_dll(
  "lib/${PROJECT_NAME}/dotnet/MatrixLibrary.dll"
  "lib/${PROJECT_NAME}/dotnet/SDK\ MCS\ Studio2.dll"
  )

ament_package()
