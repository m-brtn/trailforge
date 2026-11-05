# The Quench engine (https://github.com/m-brtn/Quench, MIT), pinned to an archive of a commit + SHA256 like the
# other dependencies. Declared here once for the game (cmake/Dependencies.cmake) and the asset tools (tools/texc);
# each sets QUENCH_HEADLESS and calls FetchContent_MakeAvailable(quench).
# Local engine work: configure with -DFETCHCONTENT_SOURCE_DIR_QUENCH=/path/to/engine (the archive is then not used).
FetchContent_Declare(quench
    URL https://github.com/m-brtn/Quench/archive/3dfb9f49e69bf98da3a576c489880296c39165d2.tar.gz
    URL_HASH SHA256=91dcf1b0303b71367ec6e47f17441b4d3ea2b585f3f05f274520d7128a6d7bbb
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    EXCLUDE_FROM_ALL)
