# ---------------------------------------------------------------------------
# liteaero_flight_proto — generated C++ from proto/liteaero_flight.proto
#
# protobuf is provided by Conan and found by the root CMakeLists.txt.
# protobuf_generate_cpp() and protobuf::libprotobuf are available after
# find_package(protobuf REQUIRED CONFIG) in the root.
# ---------------------------------------------------------------------------

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
    ${CMAKE_CURRENT_SOURCE_DIR}/liteaero_flight.proto
)

add_library(liteaero_flight_proto STATIC ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(liteaero_flight_proto PUBLIC protobuf::libprotobuf)

# Generated headers live in the build tree; expose them to consumers.
target_include_directories(liteaero_flight_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

target_compile_features(liteaero_flight_proto PUBLIC cxx_std_17)

# Suppress warnings from generated code — we do not own it.
target_compile_options(liteaero_flight_proto PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-w>
    $<$<CXX_COMPILER_ID:MSVC>:/w>
)
