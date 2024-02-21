cd react-robot-dashboard/my-robot-server
echo "Installing server dependencies..."
call npm install
echo "Starting server..."
start call npm start

cd ../../react-robot-dashboard/robot-dashboard
echo "Installing dashboard dependencies..."
call npm install
echo "Starting dashboard..."

call npm start

