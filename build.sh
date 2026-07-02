#!/bin/bash

clang main.c -o blu -lusb-1.0 || exit 1
