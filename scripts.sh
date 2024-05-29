# if the argument is "all_tests" then run all the tests
if [ "$1" == "panda_sweep" ]; then
    echo "Running all tests"
    python3 scripts/whole_test.py --min-gravity-int 16 --max-gravity-int 16 --min-gravity-frac 16 --max-gravity-frac 16 --min-fd-int 6 --max-fd-int 12 --min-fd-frac 6 --max-fd-frac 12
elif [ "$1" == "panda" ]; then
    echo "Running panda trajectory tracking"
    make
    ./bin/trajectory_tracking ../mujoco_menagerie/franka_emika_panda/scene.xml
else 
    echo "Wrong argument: $1"
fi
