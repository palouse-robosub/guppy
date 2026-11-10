if output=$(cansend can0 001#ffffffff 2>&1); then
    echo "works"
else
    echo "failed: $output"
fi