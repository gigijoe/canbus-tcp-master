#!/bin/sh
# Generate for python
lcm-gen -p ./*.lcm

# Generate for c++
lcm-gen -x ./*.lcm
