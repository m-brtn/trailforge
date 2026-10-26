# CTest level_blender_roundtrip: import a baked level into Blender, export it again without edits, then require the
# same files byte for byte and the same bake (trailforge_level_tests level_baked_compare).
# -D BLENDER=... SOURCE_DIR=... LEVEL=<id> WORK=<dir> TESTS=<trailforge_level_tests>
set(src "${SOURCE_DIR}/assets/levels/${LEVEL}")
set(out "${WORK}/levels/${LEVEL}")
file(REMOVE_RECURSE "${WORK}")
file(MAKE_DIRECTORY "${out}")
# The copied level.json reads its models from the source tree.
file(READ "${src}/level.json" level_json)
string(REPLACE "\"../../models\"" "\"${SOURCE_DIR}/assets/models\"" level_json "${level_json}")
file(WRITE "${out}/level.json" "${level_json}")

execute_process(COMMAND "${BLENDER}" -b --factory-startup --python-exit-code 1
                        --python "${SOURCE_DIR}/tools/levels/blender/import_level.py" --
                        --level "${src}" --out "${WORK}/${LEVEL}.blend"
                RESULT_VARIABLE rc OUTPUT_VARIABLE log ERROR_VARIABLE log)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "import failed (${rc}):\n${log}")
endif()
execute_process(COMMAND "${BLENDER}" -b "${WORK}/${LEVEL}.blend" --python-exit-code 1
                        --python "${SOURCE_DIR}/tools/levels/blender/export_level.py" -- --out "${out}"
                RESULT_VARIABLE rc OUTPUT_VARIABLE log ERROR_VARIABLE log)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "export failed (${rc}):\n${log}")
endif()
string(REGEX MATCH "TF: exported[^\n]*" exported "${log}")
message(STATUS "${exported}")
foreach(f heightmap.pgm surfaces.pgm layout.json)
    execute_process(COMMAND "${CMAKE_COMMAND}" -E compare_files "${src}/${f}" "${out}/${f}" RESULT_VARIABLE same)
    if(NOT same EQUAL 0)
        message(FATAL_ERROR "${f} differs after import -> export (${out}/${f})")
    endif()
    file(SIZE "${out}/${f}" size)
    message(STATUS "${f}: identical (${size} bytes)")
endforeach()
execute_process(COMMAND "${TESTS}" level_baked_compare "${src}/level.json" "${out}/level.json"
                RESULT_VARIABLE rc OUTPUT_VARIABLE log ERROR_VARIABLE log)
message(STATUS "${log}")
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "the bakes differ")
endif()
