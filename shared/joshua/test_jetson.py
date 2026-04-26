

##########################################################################################################
# Test PlanPaths
# Status: Passed
# assume the map is 100x100 pixels
from jetson import plan_paths, get_next_waypoints

def test_plan_paths():
    end = [90,90]
    start = [10,10] # lead
    f1 = [5,5]      # follower 1
    f2 = [0,0]      # follower 2

    paths = plan_paths(start, end, f1, f2)
    print(f"path leader:\n{paths[0]}")
    print(f"path follower 1:\n{paths[1]}")
    print(f"path follower 2:\n{paths[2]}")
    print("\n\n")
    

##########################################################################################################
# Test get_next_waypoints
# Status: Passed

def test_get_next_waypoints():
    end = [90,90]
    start = [10,10] # lead
    f1 = [5,5]      # follower 1
    f2 = [0,0]      # follower 2
    
    PATHS = plan_paths(start, end, f1, f2)

    #################################################################
    # time 0
    wp = get_next_waypoints(0, start)
    print(f"First five for leader: {wp}")

    wp = get_next_waypoints(1, f1)
    print(f"First five for follower 1: {wp}")

    wp = get_next_waypoints(2, f2)
    print(f"First five for follower 2: {wp}")
    print("\n\n")

    #################################################################
    # time +
    wp = get_next_waypoints(0, PATHS[0][5])
    print(f"Second five for leader: {wp}")

    wp = get_next_waypoints(1, PATHS[1][5])
    print(f"Second five for follower 1: {wp}")

    wp = get_next_waypoints(2, PATHS[2][5])
    print(f"Second five for follower 2: {wp}") 
    print("\n\n")

    #################################################################
    # time ++
    wp = get_next_waypoints(0, PATHS[0][10])
    print(f"Third five for leader: {wp}")

    wp = get_next_waypoints(1,  PATHS[1][10])
    print(f"Third five for follower 1: {wp}")

    wp = get_next_waypoints(2,  PATHS[2][10])
    print(f"Third five for follower 2: {wp}")
    print("\n\n")

    #################################################################
    # last 5 test
    wp = get_next_waypoints(0, PATHS[0][-5])
    print(f"Last five for leader: {wp}")

    wp = get_next_waypoints(1,  PATHS[1][-5])
    print(f"Last five for follower 1: {wp}")

    wp = get_next_waypoints(2,  PATHS[2][-5])
    print(f"Last five for follower 2: {wp}")
    print("\n\n")

    #################################################################
    # last 3 test
    wp = get_next_waypoints(0, PATHS[0][-3])
    print(f"Last three for leader: {wp}")

    wp = get_next_waypoints(1,  PATHS[1][-3])
    print(f"Last three for follower 1: {wp}")

    wp = get_next_waypoints(2,  PATHS[2][-3])
    print(f"Last three for follower 2: {wp}")
    print("\n\n")

   

# End Task 5
##########################################################################################################



##########################################################################################################
# main testing selection
#       consider placing the outputs of this program into another file
#       python test_jetson.py > test.txt
if __name__ == "__main__":
    test_plan_paths()
    test_get_next_waypoints()