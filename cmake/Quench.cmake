# The Quench engine (https://github.com/m-brtn/Quench, MIT), pinned to an archive of a commit + SHA256 like the
# other dependencies. Declared here once for the game (cmake/Dependencies.cmake) and the asset tools (tools/texc);
# each sets QUENCH_HEADLESS and calls FetchContent_MakeAvailable(quench).
# Local engine work: configure with -DFETCHCONTENT_SOURCE_DIR_QUENCH=/path/to/engine (the archive is then not used).
FetchContent_Declare(quench
    URL https://github.com/m-brtn/Quench/archive/dc22eeb901fe799944639cbb55512c065945b035.tar.gz
    URL_HASH SHA256=6c91c3b5e4ec4a641da655d89b4e4347f1da591c212d4057cb2bc4ff53c6b27e
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    EXCLUDE_FROM_ALL)
