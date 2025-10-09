#!/bin/bash

if ls *.java >/dev/null 2>&1; then
    result=$(java P1main Alt "$TESTDIR/../../PROBS/prob1" verbose)
else
        result=$(python3 P1main.py Alt "$TESTDIR/../../PROBS/prob1" verbose)
fi

echo "$result"

result="${result//$'\n'/NEWLINE}"

result="${result// /}"

java -jar $TESTDIR/../CheckOutput.jar $result verbose



