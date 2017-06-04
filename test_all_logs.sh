#!/bin/bash

# Since the visualizer script will print out the max length of return_path, this script can help to figure out how good the pruning is.
# For instance, suppose that we want our pruning to reduce the length to less than some number x.
# Obviously that is not always possible, but this script will test many different flight paths to get a sense of how well the pruning is performing in different circumstances.
# It's a bit difficult to stop this script after it's been started.

for f in ./logs/*.log ; do ./visualizer.py "$f" ; done
