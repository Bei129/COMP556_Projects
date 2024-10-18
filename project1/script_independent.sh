#!/bin/bash

# Define server details
SERVER="zorite.clear.rice.edu"
PORT=18227

# Define different sizes and counts
SIZES=($(seq 1000 200 65000))

# Launch clients with different sizes and counts
for i in "${!SIZES[@]}"; do
    ./client "$SERVER" "$PORT" "${SIZES[$i]}" "20"
done

echo "All clients have completed."