#!/usr/bin/bash

echo "Starting the React Robot Dashboard"

(cd my-robot-server && npm start) &

(cd robot-dashboard && npm run build && serve -s build)