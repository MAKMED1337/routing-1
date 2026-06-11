cmake_minimum_required(VERSION 3.24)
project(transport_routing LANGUAGES CXX)

set(CMAKE_CXX_EXTENSIONS OFF)


include(FetchContent)
include(CTest)

FetchContent_Declare(
  protozero
  GIT_REPOSITORY https://github.com/mapbox/protozero.git
  GIT_TAG v1.7.1
  SOURCE_SUBDIR cmake-no-add-subdirectory
)
FetchContent_Declare(
  libosmium
  GIT_REPOSITORY https://github.com/osmcode/libosmium.git
  GIT_TAG v2.22.0
  SOURCE_SUBDIR cmake-no-add-subdirectory
)
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
  GIT_SHALLOW TRUE
)
FetchContent_Declare(
  cli11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG v2.4.2
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(protozero libosmium nlohmann_json cli11)

find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
find_package(BZip2 REQUIRED)
find_package(EXPAT REQUIRED)

add_library(project_warnings INTERFACE)
target_compile_options(project_warnings INTERFACE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wconversion>
)

add_library(transport_core
  src/algorithms/alt/alt.cpp
  src/algorithms/alt/landmarks.cpp
  src/algorithms/astar.cpp
  src/io/ch_io.cpp
  src/algorithms/ch/contraction_hierarchy.cpp
  src/algorithms/ch/node_order.cpp
  src/algorithms/ch/witness_search.cpp
  src/algorithms/ch/work_graph.cpp
  src/algorithms/arcflags/arc_flags.cpp
  src/io/arc_flags_io.cpp
  src/algorithms/chase/chase.cpp
  src/algorithms/hl/hub_labels.cpp
  src/algorithms/bidirectional_astar.cpp
  src/algorithms/bidirectional_dijkstra.cpp
  src/algorithms/dijkstra.cpp
  src/algorithms/partition.cpp
  src/algorithms/phast.cpp
  src/graph/geometry.cpp
  src/graph/graph.cpp
  src/io/graph_io.cpp
  src/graph/reverse_graph.cpp
  src/algorithms/routing_instance.cpp
  src/algorithms/shortest_paths.cpp
)
target_compile_features(transport_core PUBLIC cxx_std_23)
target_include_directories(transport_core PUBLIC src)
target_link_libraries(transport_core PUBLIC Threads::Threads PRIVATE project_warnings)

find_library(KAMINPAR_LIBRARY NAMES kaminpar kaminpar-shm)
find_path(KAMINPAR_INCLUDE_DIR NAMES kaminpar-shm/kaminpar.h)
if(KAMINPAR_LIBRARY AND KAMINPAR_INCLUDE_DIR)
  message(STATUS "Found KaMinPar: ${KAMINPAR_LIBRARY}")
  target_compile_definitions(transport_core PUBLIC TRANSPORT_HAVE_KAMINPAR)
  target_include_directories(transport_core PRIVATE ${KAMINPAR_INCLUDE_DIR})
  target_link_libraries(transport_core PUBLIC ${KAMINPAR_LIBRARY})
else()
  message(STATUS "KaMinPar not found — PartitionMethod::KaMinPar will throw at runtime")
endif()

add_executable(transport_import_osm src/apps/import_main.cpp)
target_include_directories(transport_import_osm SYSTEM PRIVATE ${libosmium_SOURCE_DIR}/include ${protozero_SOURCE_DIR}/include)
target_link_libraries(transport_import_osm PRIVATE transport_core CLI11::CLI11 project_warnings ZLIB::ZLIB BZip2::BZip2 EXPAT::EXPAT)

add_executable(transport_query src/apps/query_main.cpp)
target_link_libraries(transport_query PRIVATE transport_core CLI11::CLI11 project_warnings)

add_executable(transport_benchmark src/apps/benchmark_main.cpp)
target_link_libraries(transport_benchmark PRIVATE bench_harness CLI11::CLI11)

if(BUILD_TESTING)
  add_executable(graph_io_tests tests/graph_io_test.cpp)
  target_link_libraries(graph_io_tests PRIVATE transport_core project_warnings)
  add_executable(reverse_graph_tests tests/reverse_graph_test.cpp)
  target_link_libraries(reverse_graph_tests PRIVATE transport_core project_warnings)
  add_executable(transport_routing_tests tests/routing_algorithms_test.cpp)
  target_link_libraries(transport_routing_tests PRIVATE transport_core project_warnings)
  add_executable(routing_instance_tests tests/routing_instance_test.cpp)
  target_link_libraries(routing_instance_tests PRIVATE transport_core project_warnings)
  add_executable(alt_tests tests/algorithms/alt/alt_test.cpp)
  target_include_directories(alt_tests PRIVATE tests)
  target_link_libraries(alt_tests PRIVATE transport_core project_warnings)
  add_executable(astar_heuristics_tests tests/astar_heuristics_test.cpp)
  target_link_libraries(astar_heuristics_tests PRIVATE transport_core project_warnings)
  add_executable(partition_tests tests/partition_test.cpp)
  target_link_libraries(partition_tests PRIVATE transport_core project_warnings)
  add_executable(phast_tests tests/algorithms/phast_test.cpp)
  target_include_directories(phast_tests PRIVATE tests)
  target_link_libraries(phast_tests PRIVATE transport_core project_warnings)
  add_executable(chase_tests tests/algorithms/chase/chase_test.cpp)
  target_include_directories(chase_tests PRIVATE tests)
  target_link_libraries(chase_tests PRIVATE transport_core project_warnings)
  add_executable(hl_tests tests/algorithms/hl/hub_labels_test.cpp)
  target_include_directories(hl_tests PRIVATE tests)
  target_link_libraries(hl_tests PRIVATE transport_core project_warnings)
  add_executable(arcflags_tests tests/algorithms/arcflags/arc_flags_test.cpp)
  target_include_directories(arcflags_tests PRIVATE tests)
  target_link_libraries(arcflags_tests PRIVATE transport_core project_warnings)
  add_executable(ch_io_tests tests/algorithms/ch/ch_io_test.cpp)
  target_include_directories(ch_io_tests PRIVATE tests)
  target_link_libraries(ch_io_tests PRIVATE transport_core project_warnings)
endif()

add_executable(write_benchmark_graph src/apps/write_benchmark_graph.cpp)
target_link_libraries(write_benchmark_graph PRIVATE transport_core CLI11::CLI11 project_warnings)

# Shared interface for all benchmark binaries: nlohmann/json.
add_library(bench_harness INTERFACE)
target_link_libraries(bench_harness INTERFACE transport_core nlohmann_json::nlohmann_json project_warnings)

add_executable(ch_measure src/apps/ch_measure_main.cpp)
target_link_libraries(ch_measure PRIVATE bench_harness CLI11::CLI11)

add_executable(transport_query_gen src/apps/query_gen_main.cpp)
target_link_libraries(transport_query_gen PRIVATE bench_harness CLI11::CLI11)

add_executable(transport_bench_replay src/apps/bench_replay_main.cpp)
target_link_libraries(transport_bench_replay PRIVATE bench_harness CLI11::CLI11)

if(BUILD_TESTING)
  add_test(NAME graph_io COMMAND graph_io_tests)
  add_test(NAME reverse_graph COMMAND reverse_graph_tests)
  add_test(NAME query_help COMMAND transport_query --help)
  add_test(NAME import_help COMMAND transport_import_osm --help)
  add_test(NAME benchmark_help COMMAND transport_benchmark --help)
  add_test(NAME write_benchmark_graph_help COMMAND write_benchmark_graph --help)
  add_test(NAME ch_measure_help COMMAND ch_measure --help)
  add_test(NAME query_gen_help COMMAND transport_query_gen --help)
  add_test(NAME bench_replay_help COMMAND transport_bench_replay --help)
  add_test(NAME routing_algorithms COMMAND transport_routing_tests)
  add_test(NAME routing_instance COMMAND routing_instance_tests)
  add_test(NAME alt COMMAND alt_tests)
  add_test(NAME astar_heuristics COMMAND astar_heuristics_tests)
  add_test(NAME partition COMMAND partition_tests)
  add_test(NAME phast COMMAND phast_tests)
  add_test(NAME chase COMMAND chase_tests)
  add_test(NAME hl COMMAND hl_tests)
  add_test(NAME arcflags COMMAND arcflags_tests)
  add_test(NAME ch_io COMMAND ch_io_tests)
endif()
