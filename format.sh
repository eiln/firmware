#!/usr/bin/env sh

for dir in common/phal_G4 source/g4_testing
do
    find "$dir" -type f \( -iname '*.c' -o -iname '*.h' \) -exec clang-format -style=file -assume-filename="/home/eileen/per/firmware/.clang-format" -i {} +
done
