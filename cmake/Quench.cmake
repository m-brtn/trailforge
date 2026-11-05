# The Quench engine (https://github.com/m-brtn/Quench, MIT), pinned to an archive of a commit + SHA256 like the
# other dependencies. Declared here once for the game (cmake/Dependencies.cmake) and the asset tools (tools/texc);
# each sets QUENCH_HEADLESS and calls FetchContent_MakeAvailable(quench).
# Local engine work: configure with -DFETCHCONTENT_SOURCE_DIR_QUENCH=/path/to/engine (the archive is then not used).
FetchContent_Declare(quench
    URL https://github.com/m-brtn/Quench/archive/d70878320f35c2c5b455666e92bb5dd2f5d86cd1.tar.gz
    URL_HASH SHA256=8f77c4f566eb87facfe34fab585275eaab04963687e816ab8e8629ce90d2f4c8
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    EXCLUDE_FROM_ALL)
