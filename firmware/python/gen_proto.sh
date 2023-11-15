#!/bin/bash

set -e  # exit on error

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

python_out="$SCRIPT_DIR/generated/"

mkdir -p $python_out
protoc -I="$SCRIPT_DIR/../protoduck" --python_out=$python_out  messages.proto

echo "protobuf files generated in $python_out"

