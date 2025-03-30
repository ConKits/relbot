#!/bin/bash
cd /home/ubuntu/relBot
git add .
git commit -m "Auto-commit: $(date)"
git push origin main
