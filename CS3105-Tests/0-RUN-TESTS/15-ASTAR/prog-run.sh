#!/bin/bash

if ls *.java >/dev/null 2>&1; then
    java P1main AStar "$TESTDIR/../../PROBS/prob5"
else
    python3 P1main.py AStar "$TESTDIR/../../PROBS/prob5"
fi

