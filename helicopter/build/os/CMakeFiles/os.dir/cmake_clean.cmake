FILE(REMOVE_RECURSE
  "libos.pdb"
  "libos.a"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/os.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
