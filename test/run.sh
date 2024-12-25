#!/bin/bash

EXE="../build/MBOW"
ENV_FILE=$1

$EXE $(pwd)/$ENV_FILE
python="/home/airlab/anaconda3/envs/limbo/bin/python"
$python result_line.py --env=$ENV_FILE --result="result.csv"