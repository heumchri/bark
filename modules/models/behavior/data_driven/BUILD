cc_library(
    name = "data_driven",
    srcs = [
        "data_driven.cpp",
    ],
    hdrs = [
        "data_driven.hpp",
    ],
    deps = [
        "//modules/commons:commons",
        "//modules/world:world",
        "//modules/models/behavior:behavior",
        "//modules/models/dynamic:dynamic"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
