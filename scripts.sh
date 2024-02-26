# if the argument is "all_tests" then run all the tests
if [ "$1" == "tests" ]; then
    echo "Running all tests"
    python3 scripts/whole_test.py
elif [ "$1" == "panda" ]; then
    echo "Running panda trajectory tracking"
    ./bin/trajectory_tracking ../mujoco_menagerie/franka_emika_panda/scene.xml
else 
    echo "Wrong argument: $1"
fi
