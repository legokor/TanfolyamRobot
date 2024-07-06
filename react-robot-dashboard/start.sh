#!/usr/bin/bash

echo "Starting the React Robot Dashboard"

(cd /WS/Tanfrobot/TanfolyamRobot/react-robot-dashboard/my-robot-server && npm start) &

(cd /WS/Tanfrobot/TanfolyamRobot/react-robot-dashboard/robot-dashboard && npm start)