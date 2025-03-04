#!/bin/bash

# Define server details
SERVER="zorite.clear.rice.edu"
PORT=18227

# Define different sizes and counts
SIZES=(1000 2000 3000 4000 5000 20)
COUNTS=(5 10 10 20 8 2000)

# Launch clients with different sizes and counts
for i in "${!SIZES[@]}"; do
  ./client "$SERVER" "$PORT" "${SIZES[$i]}" "${COUNTS[$i]}" &
done

wait

echo "All clients have completed."