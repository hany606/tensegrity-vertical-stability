link_directories(${LIB_DIR})

#find_package(nlohmann_json REQUIRED)

link_libraries(tgcreator
                util
                sensors
                core
                terrain 
                tgOpenGLSupport)

#add_subdirectory(json)
set(JSON_BuildTests OFF CACHE INTERNAL "")

add_subdirectory(json)
include_directories(/json/include/nlohmann)

add_executable(AppJumperModel
    JumperModel.cpp
    AppJumperModel.cpp
    LengthController.cpp
    TCP.cpp
    JSON_Structure.cpp
) 


target_link_libraries(AppJumperModel PRIVATE nlohmann_json::nlohmann_json)
