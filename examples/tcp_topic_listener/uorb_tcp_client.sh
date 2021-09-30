#!/usr/bin/env bash

# stty -echo -icanon && nc localhost 10924

echo "$*" | nc localhost 10924
