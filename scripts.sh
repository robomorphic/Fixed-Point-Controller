# if the argument is "all_tests" then run all the tests
if [ "$1" == "panda_sweep" ]; then
    echo "Running all tests"
    python3 scripts/whole_test.py --min-gravity-int 15 --max-gravity-int 15 --min-gravity-frac 15 --max-gravity-frac 15 --min-fd-int 8 --max-fd-int 10 --min-fd-frac 8 --max-fd-frac 10
elif [ "$1" == "panda" ]; then
    echo "Running panda trajectory tracking"
    make
    ./bin/trajectory_tracking ../mujoco_menagerie/franka_emika_panda/scene.xml
else 
    echo "Wrong argument: $1"
fi