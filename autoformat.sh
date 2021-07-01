#!/bin/sh
find . -type f -name '*.h' -o -iname '*.c' -o -iname '*.cpp' -o -iname '*.hpp' \
    | grep -v -e build/ -e submodules/ \
    | xargs clang-format --verbose -style=file -i -fallback-style=none

black .
