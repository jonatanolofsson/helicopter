#!/bin/sh
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ROOT="$(dirname "$DIR")"
APPSYSDIR="$(pwd)"
for module_config in "$@"
do
    module=${module_config##*/}
    module=${module%.*}
    module=${module,,}
    if [ -d "${ROOT}/sys/${module}" ]; then
        echo ": \$(TOP)/sys/${module}/*.cpp |> !cc -o ${module}.o |> ${module}.o"
    fi
    if [ -d "$APPSYSDIR/${module}" ]; then
        echo ": ${module}/*.cpp |> !cc -o ${module}.o |> ${module}.o"
    fi
done
