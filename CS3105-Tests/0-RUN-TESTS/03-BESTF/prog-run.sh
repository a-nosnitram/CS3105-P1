#!/bin/bash

if ls *.java >/dev/null 2>&1; then
    java P1main BestF "$TESTDIR/../../PROBS/prob3"
else
    python3 P1main.py BestF "$TESTDIR/../../PROBS/prob3"
fi

