echo "mkdir -p ~/scripts" >&2
ssh pi@car10 mkdir -p scripts
echo "scp'ing..." >&2
scp -r . pi@car10:scripts/