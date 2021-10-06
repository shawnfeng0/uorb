#!/usr/bin/env bash

# stty -echo -icanon && nc -v localhost 10924

echo "$*" | nc -v localhost 10924
