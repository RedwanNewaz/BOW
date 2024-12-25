#!/bin/bash

EXE="../build/MBOW"
ENV_FILE=$1

$EXE $(pwd)/$ENV_FILE
python="/home/airlab/anaconda3/envs/limbo/bin/python"
$python mresult_line.py --env=$ENV_FILE --results  "$(pwd)/1_result.csv, $(pwd)/2_result.csv"