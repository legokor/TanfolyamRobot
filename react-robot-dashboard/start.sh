#!/usr/bin/bash

echo "Starting the React Robot Dashboard"

(cd my-robot-server && npm start) &

(cd robot-dashboard && serve -s build)