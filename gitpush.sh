#!/bin/bash
git add .
echo "Files staged."
git commit -m "$1"
echo "Committed with message: $1"
git push
echo "Pushed to GitHub."
