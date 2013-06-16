FILE(REMOVE_RECURSE
  "libmath.pdb"
  "libmath.a"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/math.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
