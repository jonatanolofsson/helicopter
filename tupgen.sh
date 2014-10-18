#!/bin/sh
for module_config in "$@"
do
    module=${module_config##*/}
    module=${module%.*}
    module=${module,,}
    if [ -d "sys/${module}" ]; then
        echo ": sys/${module}/*.cpp |> !cc |> ${module}.o"
    fi
done
