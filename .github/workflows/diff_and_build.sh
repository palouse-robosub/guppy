#!/usr/bin/env bash

PKGS=$(colcon list --names-only)

CHANGED_FILES=$(git diff --name-only HEAD~1...HEAD)

CHANGED_PKGS=$(for i in $CHANGED_FILES; do
    for j in $PKGS; do
        if [[ $i == *$j* && $j != "guppy" ]]; then 
            printf "$j\n"
            break
        fi
    done
done | sort -u)

echo Changed pkgs: $CHANGED_PKGS

colcon build --packages-above-and-dependencies $CHANGED_PKGS
colcon test --packages-above-and-dependencies $CHANGED_PKGS