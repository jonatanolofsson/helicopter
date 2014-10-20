#!/bin/sh
for module_config in "$@"
do
    module=${module_config##*/}
    module=${module%.*}
    module=${module,,}
    if [ -d "../../../sys/${module}" ]; then
        echo ": \$(TOP)/sys/${module}/*.cpp |> !cc -o ${module}.o |> ${module}.o"
    fi
done
